#
# Makefile for the Sun network device drivers.
#

obj-$(CONFIG_HAPPYMEAL) += sunhme.o
obj-$(CONFIG_SUNQE) += sunqe.o
obj-$(CONFIG_SUNBMAC) += sunbmac.o
obj-$(CONFIG_SUNGEM) += sungem.o
obj-$(CONFIG_CASSINI) += cassini.o
obj-$(CONFIG_SUNVNET) += sunvnet.o
obj-$(CONFIG_NIU) += niu.o
obj-$(CONFIG_OPENSPARC_NET)	+= sunhv_net_mod.o

sunhv_net_mod-y	:= sunhv_net.o sunhv_net_hcall.o
