# Copyright (c) 2018 - Adjacent Link LLC, Bridgewater, New Jersey
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of Adjacent Link LLC nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# See toplevel COPYING for more information.

"""Multicast communication channel.

"""
from __future__ import absolute_import, division, print_function
from .channel import Channel
import socket
import traceback
import sys
import fcntl
import struct
import ctypes
import logging

def get_ip_address(ifname):
    # http://code.activestate.com/recipes/439094/
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15].encode() if sys.version_info >= (3,0) else ifname[:15])
    )[20:24])

class MulticastChannel(Channel):
    def __init__(self,ctx,**kwargs):
        """Creates the channel instance. Supports IPv4 and IPv6.

        Args:
          ctx (obj): Context instance.

        Keyword Args:
          group (str): Mutlicast group address.

          group_port (int): Mutlicast group port.

          device (str): Device to associate with the multicast
            group. Default None.

          tos (int): IP tos value for transmitted packets. IPv4
            only. Default 0.

          loop (bool): Enable multicast loop. Default False.

          ttl (int): Packet TTL. Default 1.

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

        Raises:
            KeyError: If an invalid keyword is found.

        """
        super(MulticastChannel, self).__init__(ctx)

        self.__group = kwargs.pop('group',None)

        group_port = kwargs.pop('group_port',None)

        self.__device = kwargs.pop('device',None)

        self._tos = kwargs.pop('tos',0)

        self._loop = kwargs.pop('loop',False)

        self._ttl = kwargs.pop('ttl',1)

        self.__on_message = kwargs.pop('on_message',lambda *args : None)

        if self.__group == None:
            raise KeyError('{} missing kwarg: group'.format(self.__class__.__name__))

        if group_port == None:
            raise KeyError('{} missing kwarg: group_port'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))

        self.__addr_info = socket.getaddrinfo(self.__group,group_port,0,0,socket.SOL_UDP)

    def start(self):
        """Starts the channel.

        Raises:
            RuntimeError: If an socket or socket option error occurs.

            IOError: If an unknown device was specified.

        """
        af_type = self.__addr_info[0][0]

        self.__socket = socket.socket(af_type,socket.SOCK_DGRAM,0)

        try:
            if af_type == socket.AF_INET6:
                self.__socket.setsockopt(socket.IPPROTO_IPV6,socket.IPV6_MULTICAST_HOPS,
                                         self._ttl)
            else:
                self.__socket.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,
                                         self._ttl)

        except socket.error as msg :
            if sys.version_info >= (3,3):
                raise RuntimeError('{} socket option failure {}'.format(self.__class__.__name__,
                                                                        str(msg)))
            else:
                raise RuntimeError('{} socket option failure {} {}'.format(self.__class__.__name__,
                                                                           str(msg[0]),
                                                                           msg[1]))
        try:
            if af_type == socket.AF_INET6:
                self.__socket.setsockopt(socket.IPPROTO_IPV6,socket.IPV6_MULTICAST_LOOP,
                                         1 if self._loop else 0)
            else:
                self.__socket.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_LOOP,
                                         1 if self._loop else 0)
        except socket.error as msg :
            if sys.version_info >= (3,3):
                raise RuntimeError('{} socket option failure {}'.format(self.__class__.__name__,
                                                                        str(msg)))
            else:
                raise RuntimeError('{} socket option failure {} {}'.format(self.__class__.__name__,
                                                                           str(msg[0]),
                                                                           msg[1]))

        try:
            self.__socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR, 1)
        except socket.error as msg :
            if sys.version_info >= (3,3):
                raise RuntimeError('{} socket option failure {}'.format(self.__class__.__name__,
                                                                        str(msg)))
            else:
                raise RuntimeError('{} socket option failure {} {}'.format(self.__class__.__name__,
                                                                           str(msg[0]),
                                                                           msg[1]))

        try:
            if af_type == socket.AF_INET:
                self.__socket.setsockopt(socket.SOL_IP,socket.IP_TOS,self._tos)

        except socket.error as msg :
            if sys.version_info >= (3,3):
                raise RuntimeError('{} socket option failure {}'.format(self.__class__.__name__,
                                                                        str(msg)))
            else:
                raise RuntimeError('{} socket option failure {} {}'.format(self.__class__.__name__,
                                                                           str(msg[0]),
                                                                           msg[1]))

        try:
            self.__socket.bind(self.__addr_info[0][4])
        except socket.error as msg:
            if sys.version_info >= (3,3):
                raise RuntimeError('{} bind failure {}'.format(self.__class__.__name__,
                                                               str(msg)))
            else:
                raise RuntimeError('{} bind failure {} {}'.format(self.__class__.__name__,
                                                                  str(msg[0]),
                                                                  msg[1]))

        try:
            if af_type == socket.AF_INET6:
                if_index = struct.pack('I',0)

                if self.__device != None:
                    libc = ctypes.CDLL(None)

                    if_index = struct.pack('I',libc.if_nametoindex(ctypes.c_char_p(self.__device.encode('utf-8'))))

                    self.__socket.setsockopt(socket.IPPROTO_IPV6,
                                             socket.IPV6_MULTICAST_IF,
                                             if_index)

                mreq6 = socket.inet_pton(socket.AF_INET6,self.__group) + if_index

                self.__socket.setsockopt(socket.IPPROTO_IPV6,
                                         socket.IPV6_JOIN_GROUP,
                                         mreq6)
            else:
                if_index  = struct.pack('I',0)

                if self.__device != None:
                    libc = ctypes.CDLL(None)

                    if_index = struct.pack('I',libc.if_nametoindex(ctypes.c_char_p(self.__device.encode('utf-8'))))

                    devAddress = socket.inet_aton(get_ip_address(self.__device))
                else:
                    devAddress = socket.inet_aton("0.0.0.0")

                mreqn = socket.inet_aton(self.__group) + devAddress + if_index

                self.__socket.setsockopt(socket.SOL_IP,
                                         socket.IP_ADD_MEMBERSHIP,
                                         mreqn)

                self.__socket.setsockopt(socket.SOL_IP,
                                         socket.IP_MULTICAST_IF,
                                         mreqn)

        except socket.error as msg:
            if sys.version_info >= (3,3):
                raise RuntimeError('{} socket add membership failure {}'.format(self.__class__.__name__,
                                                                        str(msg)))
            else:
                raise RuntimeError('{} socket mulicast add membership failure {} {}'.format(self.__class__.__name__,
                                                                                            str(msg[0]),
                                                                                            msg[1]))
        except IOError:
            raise RuntimeError('{} unknown device {}'.format(self.__class__.__name__,
                                                             self.__device))


        self.ctx.add_fd(self.__socket.fileno(),self.read)


    def read(self):
        """Reads from the channel and calls on_message callable.

        """

        data,_ = self.__socket.recvfrom(65535)

        if len(data):
            try:
                self.__on_message(self.ctx,self.id,data)
            except:
                logging.error(traceback.format_exc())

    def send(self,data,**kwargs):
        """Sends a message over the channel.

        Args:
          data (str): Data to send.

        """
        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))


        self.__socket.sendto(data,self.__addr_info[0][4])

    def destroy(self):
        """Destroys the channel.

        """
        self.ctx.remove_fd(self.__socket.fileno())
        self.__socket.close()
