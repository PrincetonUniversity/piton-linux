/*
 * mitts.c: System calls for setting or getting a process's MITTS'
 *          configuration register values.
 *
 * Copyright (C) 2016 Princeton University
 */

#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/mitts.h>

/*
 * Extracts information from cfg and places the corresponding
 * values for MITTS config registers 1 and 2 into reg1 and reg2.
 */
static void set_config_regs(struct mitts_cfg *cfg, unsigned long *reg1,
		unsigned long *reg2)
{
	unsigned long r1, r2;
	int i;

	r1 = 0;
	r2 = 0;

	/* Set R1 */
	for (i = N_BINS-1; i > 0; i--) {
		r1 = r1 | (cfg->bins[i] & BIN_MASK);
		r1 = r1 << BIN_SHMT;
	}
	r1 = r1 | (cfg->bins[0] & BIN_MASK);
	r1 = r1 << CFG_SHMT;
	if (cfg->en) {
		r1 = r1 | ENABLED;
	}
	*reg1 = r1;

	/* Set R2 */
	r2 = r2 | (cfg->scale & SCALE_MASK);
	r2 = r2 << SCALE_SHMT;
	r2 = r2 | cfg->interval;
	*reg2 = r2;

	return;
}

/*
 * Extracts information from reg1 and reg2, which store the
 * contents of their respective MITTS configuration registers,
 * and initializes cfg with the corresponding data.
 */
static void get_config_struct(struct mitts_cfg *cfg, unsigned long reg1,
		unsigned long reg2)
{
	int i;

	/* Read R1 */
	cfg->en = reg1 & EN_MASK;
	for (i = 0; i < N_BINS; i++) {
		cfg->bins[i] = ((reg1 >> CFG_SHMT) >> (BIN_SHMT * i))
				& BIN_MASK;
	}

	/* Read R2 */
	cfg->interval = reg2 & INT_MASK;
	cfg->scale = (reg2 >> SCALE_SHMT) & SCALE_MASK;

	return;
}

/*
 * Uses the contents of cfg to properly set up task p's MITTS
 * configuration registers.
 */
static void set_task_mittscfg(struct task_struct *p, struct mitts_cfg *cfg)
{
	struct thread_info *ti;

	ti = task_thread_info(p);
	set_config_regs(cfg, &ti->mitts_cfg[0], &ti->mitts_cfg[1]);

	return;
}

/*
 * Returns true if current's euid is same as p's uid or euid,
 * or has CAP_SYS_NICE to p's user_ns.
 *
 * Called with rcu_read_lock, creds are safe
 */
static bool has_mittscfg_perm(struct task_struct *p)
{
	const struct cred *cred = current_cred(), *pcred = __task_cred(p);

	if (uid_eq(pcred->uid,  cred->euid) ||
	    uid_eq(pcred->euid, cred->euid)) {
		return true;
	}
	if (ns_capable(pcred->user_ns, CAP_SYS_NICE)) {
		return true;
	}
	return false;
}

/*
 * Returns true if the current user has CAP_SYS_NICE for all processes
 * or if the new MITTS configuration, cfg, is strictly less than or
 * equal to task p's configuration in performance.
 *
 * Called with rcu_read_lock.
 */
static bool can_modify_mittscfg(struct task_struct *p, struct mitts_cfg *cfg)
{
	struct mitts_cfg old_cfg;
	struct thread_info *ti;
	int i;

	if (capable(CAP_SYS_NICE)) {
		return true;
	}

	ti = task_thread_info(p);
	get_config_struct(&old_cfg, ti->mitts_cfg[0], ti->mitts_cfg[1]);

	if (old_cfg.en && !cfg->en)
		return false;
	if (cfg->en && old_cfg.en) {
		if (old_cfg.scale > cfg->scale)
			return false;
		if (old_cfg.interval > cfg->interval)
			return false;
		for (i = 0; i < N_BINS; i++) {
			if (old_cfg.bins[i] < cfg->bins[i])
				return false;
		}
	}

	return true;
}

/*
 * Uses cfg to set the appropriate MITTS configuration registers
 * for task p.
 *
 * Returns EPERM if the current user has no permission
 * to modify MITTS configurations for task p.
 * Returns EACCES if the current user is attempting to increase
 * the performance allowed by MITTS but is not the superuser.
 * Otherwise, returns the original error passed or zero if
 * the original error was ERSCH.
 */
static int set_one_mittscfg(struct task_struct *p, struct mitts_cfg *cfg, int error)
{

	if (!has_mittscfg_perm(p)) {
		error = -EPERM;
		goto out;
	}
	if (!can_modify_mittscfg(p, cfg)) {
		error = -EACCES;
		goto out;
	}
	if (error == -ESRCH)
		error = 0;
	set_task_mittscfg(p, cfg);
out:
	return error;
}

/*
 * Updates the configuration in best_cfg based on the MITTS configuration of
 * task p.
 * This function seeks to fill best_cfg with the most permissive
 * MITTS configuration that can be created by combining p's config
 * and best_cfg.
 * The most permissive is a disabled mitts configuration,
 * in which case best_cfg will be zeroed out.
 * Otherwise, each bin is filled with the maximal value between the two
 * configs, and interval and scale are filled with the minimal value
 * between the two configs.
 */
static void update_best_mittscfg(struct task_struct *p, struct mitts_cfg *best_cfg)
{
	struct mitts_cfg cfg;
	struct thread_info *ti;
	int i;

	ti = task_thread_info(p);
	get_config_struct(&cfg, ti->mitts_cfg[0], ti->mitts_cfg[1]);

	if (!cfg.en && best_cfg->en) {
		memset(best_cfg, 0, sizeof(struct mitts_cfg));
		return;
	}
	if (cfg.scale < best_cfg->scale)
		best_cfg->scale = cfg.scale;
	if (cfg.interval < best_cfg->interval)
		best_cfg->interval = cfg.interval;
	for (i = 0; i < N_BINS; i++) {
		if (cfg.bins[i] > best_cfg->bins[i])
			best_cfg->bins[i] = cfg.bins[i];
	}

	return;
}

/*
 * mitts_setcfg syscall
 *
 * which | pid, pgid, or uid of target
 * who   | PRIO_PROCESS, PRIO_PGRP, or PRIO_USER_
 * cfg   | pointer to struct mitts_cfg
 *
 * Uses cfg to set the MITTS configuration of the process, process group,
 * or user process group specified by which and who.
 *
 * returns 0 on success, other on failure.
 */
SYSCALL_DEFINE3(mitts_setcfg, int, which, int, who, struct mitts_cfg __user *, cfg)
{
	struct task_struct *g, *p;
	struct user_struct *user;
	struct mitts_cfg new_cfg;
	const struct cred *cred = current_cred();
	int error = -EINVAL;
	struct pid *pgrp;
	kuid_t uid;

	if (which > PRIO_USER || which < PRIO_PROCESS)
		goto out;

	error = -EFAULT;

	/* Read the user-space cfg struct */
	if (copy_from_user(&new_cfg, cfg, sizeof(struct mitts_cfg)))
		goto out;

	error = -ESRCH;

	rcu_read_lock();
	read_lock(&tasklist_lock);
	switch (which) {
	case PRIO_PROCESS:
		if (who)
			p = find_task_by_vpid(who);
		else
			p = current;
		if (p)
			error = set_one_mittscfg(p, &new_cfg, error);
		break;
	case PRIO_PGRP:
		if (who)
			pgrp = find_vpid(who);
		else
			pgrp = task_pgrp(current);
		do_each_pid_thread(pgrp, PIDTYPE_PGID, p) {
			error = set_one_mittscfg(p, &new_cfg, error);
		} while_each_pid_thread(pgrp, PIDTYPE_PGID, p);
		break;
	case PRIO_USER:
		uid = make_kuid(cred->user_ns, who);
		user = cred->user;
		if (!who)
			uid = cred->uid;
		else if (!uid_eq(uid, cred->uid)) {
			user = find_user(uid);
			if (!user)
				goto out_unlock;	/* No process for this user */
		}
		do_each_thread(g, p) {
			if (uid_eq(task_uid(p), uid))
				error = set_one_mittscfg(p, &new_cfg, error);
		} while_each_thread(g, p);
		if (!uid_eq(uid, cred->uid))
			free_uid(user);		/* For find_user() */
		break;
	}
out_unlock:
	read_unlock(&tasklist_lock);
	rcu_read_unlock();
out:
	return error;
}

/*
 * mitts_getcfg syscall
 *
 * which | pid, pgid, or uid of target
 * who   | PRIO_PROCESS, PRIO_PGRP, or PRIO_USER_
 * cfg   | pointer to struct mitts_cfg
 *
 * Collects a mitts_cfg struct that represents the most permissive
 * MITTS configuration that can be created by combining all MITTS
 * configurations from the process, process group, or user process set
 * specified.
 *
 * returns 0 on success, other on failure.
 */
SYSCALL_DEFINE3(mitts_getcfg, int, which, int, who, struct mitts_cfg __user *, cfg)
{
	struct task_struct *g, *p;
	struct user_struct *user;
	struct mitts_cfg ret_cfg;
	const struct cred *cred = current_cred();
	long error = -ESRCH;
	struct pid *pgrp;
	kuid_t uid;

	if (which > PRIO_USER || which < PRIO_PROCESS)
		return -EINVAL;

	/*
	 * Set ret_cfg to least permissive possible configuration:
	 *  - MITTS enabled
	 *  - All Bins Zero
	 *  - Scale and Interval at Maximum Values
	 */
	memset(&ret_cfg, 0, sizeof(struct mitts_cfg));
	ret_cfg.en = 1;
	ret_cfg.scale = SCALE_MASK;
	ret_cfg.interval = INT_MASK;

	rcu_read_lock();
	read_lock(&tasklist_lock);
	switch (which) {
	case PRIO_PROCESS:
		if (who)
			p = find_task_by_vpid(who);
		else
			p = current;
		if (p) {
			update_best_mittscfg(p, &ret_cfg);
			error = 0;
		}
		break;
	case PRIO_PGRP:
		if (who)
			pgrp = find_vpid(who);
		else
			pgrp = task_pgrp(current);
		do_each_pid_thread(pgrp, PIDTYPE_PGID, p) {
			update_best_mittscfg(p, &ret_cfg);
			error = 0;
		} while_each_pid_thread(pgrp, PIDTYPE_PGID, p);
		break;
	case PRIO_USER:
		uid = make_kuid(cred->user_ns, who);
		user = cred->user;
		if (!who)
			uid = cred->uid;
		else if (!uid_eq(uid, cred->uid)) {
			user = find_user(uid);
			if (!user)
				goto out_unlock;	/* No processes for this user */
		}
		do_each_thread(g, p) {
			if (uid_eq(task_uid(p), uid)) {
				update_best_mittscfg(p, &ret_cfg);
				error = 0;
			}
		} while_each_thread(g, p);
		if (!uid_eq(uid, cred->uid))
			free_uid(user);		/* for find_user() */
		break;
	}
out_unlock:
	read_unlock(&tasklist_lock);
	rcu_read_unlock();

	if (!error && copy_to_user(cfg, &ret_cfg, sizeof(struct mitts_cfg)))
		return -EFAULT;

	return error;
}
