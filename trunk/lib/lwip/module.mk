# module.mk
# 
# STM32 DMX512
# Copyright (C) 2012 Erik Van Hamme, all rights reserved
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
# 

# This file is used for configuration of the build system.

SRCS += \
	lwip/src/api/api_lib.c \
	lwip/src/api/api_msg.c \
	lwip/src/api/err.c \
	lwip/src/api/netbuf.c \
	lwip/src/api/netdb.c \
	lwip/src/api/netifapi.c \
	lwip/src/api/sockets.c \
	lwip/src/api/tcpip.c \
	lwip/src/core/def.c \
	lwip/src/core/dhcp.c \
	lwip/src/core/dns.c \
	lwip/src/core/init.c \
	lwip/src/core/ipv4/autoip.c \
	lwip/src/core/ipv4/icmp.c \
	lwip/src/core/ipv4/igmp.c \
	lwip/src/core/ipv4/inet.c \
	lwip/src/core/ipv4/inet_chksum.c \
	lwip/src/core/ipv4/ip_addr.c \
	lwip/src/core/ipv4/ip.c \
	lwip/src/core/ipv4/ip_frag.c \
	lwip/src/core/mem.c \
	lwip/src/core/memp.c \
	lwip/src/core/netif.c \
	lwip/src/core/pbuf.c \
	lwip/src/core/raw.c \
	lwip/src/core/snmp/asn1_dec.c \
	lwip/src/core/snmp/asn1_enc.c \
	lwip/src/core/snmp/mib2.c \
	lwip/src/core/snmp/mib_structs.c \
	lwip/src/core/snmp/msg_in.c \
	lwip/src/core/snmp/msg_out.c \
	lwip/src/core/stats.c \
	lwip/src/core/sys.c \
	lwip/src/core/tcp.c \
	lwip/src/core/tcp_in.c \
	lwip/src/core/tcp_out.c \
	lwip/src/core/timers.c \
	lwip/src/core/udp.c \
	lwip/src/netif/etharp.c \
	lwip/src/netif/ethernetif.c \
	lwip/src/netif/ppp/auth.c \
	lwip/src/netif/ppp/chap.c \
	lwip/src/netif/ppp/chpms.c \
	lwip/src/netif/ppp/fsm.c \
	lwip/src/netif/ppp/ipcp.c \
	lwip/src/netif/ppp/lcp.c \
	lwip/src/netif/ppp/magic.c \
	lwip/src/netif/ppp/md5.c \
	lwip/src/netif/ppp/pap.c \
	lwip/src/netif/ppp/ppp.c \
	lwip/src/netif/ppp/ppp_oe.c \
	lwip/src/netif/ppp/randm.c \
	lwip/src/netif/ppp/vj.c \
	lwip/src/netif/slipif.c \
	lwip/STM32F4x7/Standalone/ethernetif.c \
	lwip/STM32F4x7/netconf/netconf.c

HDRS += \
	lwip/src/include/ipv4/lwip/autoip.h \
	lwip/src/include/ipv4/lwip/icmp.h \
	lwip/src/include/ipv4/lwip/igmp.h \
	lwip/src/include/ipv4/lwip/inet_chksum.h \
	lwip/src/include/ipv4/lwip/inet.h \
	lwip/src/include/ipv4/lwip/ip_addr.h \
	lwip/src/include/ipv4/lwip/ip_frag.h \
	lwip/src/include/ipv4/lwip/ip.h \
	lwip/src/include/lwip/api.h \
	lwip/src/include/lwip/api_msg.h \
	lwip/src/include/lwip/arch.h \
	lwip/src/include/lwip/debug.h \
	lwip/src/include/lwip/def.h \
	lwip/src/include/lwip/dhcp.h \
	lwip/src/include/lwip/dns.h \
	lwip/src/include/lwip/err.h \
	lwip/src/include/lwip/init.h \
	lwip/src/include/lwip/mem.h \
	lwip/src/include/lwip/memp.h \
	lwip/src/include/lwip/memp_std.h \
	lwip/src/include/lwip/netbuf.h \
	lwip/src/include/lwip/netdb.h \
	lwip/src/include/lwip/netifapi.h \
	lwip/src/include/lwip/netif.h \
	lwip/src/include/lwip/opt.h \
	lwip/src/include/lwip/pbuf.h \
	lwip/src/include/lwip/raw.h \
	lwip/src/include/lwip/sio.h \
	lwip/src/include/lwip/snmp_asn1.h \
	lwip/src/include/lwip/snmp.h \
	lwip/src/include/lwip/snmp_msg.h \
	lwip/src/include/lwip/snmp_structs.h \
	lwip/src/include/lwip/sockets.h \
	lwip/src/include/lwip/stats.h \
	lwip/src/include/lwip/sys.h \
	lwip/src/include/lwip/tcp.h \
	lwip/src/include/lwip/tcp_impl.h \
	lwip/src/include/lwip/tcpip.h \
	lwip/src/include/lwip/timers.h \
	lwip/src/include/lwip/udp.h \
	lwip/src/include/netif/etharp.h \
	lwip/src/include/netif/ppp_oe.h \
	lwip/src/include/netif/slipif.h \
	lwip/src/netif/ppp/auth.h \
	lwip/src/netif/ppp/chap.h \
	lwip/src/netif/ppp/chpms.h \
	lwip/src/netif/ppp/fsm.h \
	lwip/src/netif/ppp/ipcp.h \
	lwip/src/netif/ppp/lcp.h \
	lwip/src/netif/ppp/magic.h \
	lwip/src/netif/ppp/md5.h \
	lwip/src/netif/ppp/pap.h \
	lwip/src/netif/ppp/pppdebug.h \
	lwip/src/netif/ppp/ppp.h \
	lwip/src/netif/ppp/ppp_impl.h \
	lwip/src/netif/ppp/randm.h \
	lwip/src/netif/ppp/vj.h \
	lwip/STM32F4x7/arch/bpstruct.h \
	lwip/STM32F4x7/arch/cc.h \
	lwip/STM32F4x7/arch/cpu.h \
	lwip/STM32F4x7/arch/epstruct.h \
	lwip/STM32F4x7/arch/init.h \
	lwip/STM32F4x7/arch/lib.h \
	lwip/STM32F4x7/arch/perf.h \
	lwip/STM32F4x7/arch/sys_arch.h \
	lwip/STM32F4x7/Standalone/ethernetif.h \
	lwip/STM32F4x7/netconf/netconf.h
