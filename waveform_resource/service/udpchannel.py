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

"""Unicast communication channel.

"""
from __future__ import absolute_import, division, print_function
from .channel import Channel
import socket
import logging
import traceback

class UDPChannel(Channel):
    def __init__(self,ctx,**kwargs):
        """Creates the channel instance. Supports IPv4 and IPv6.

        Args:
          ctx (obj): Context instance.

        Keyword Args:
          local (str): Local address or hostname.

          local_port (int): Local port.

          tos (int): IP tos value for transmitted packets. IPv4
            only. Default 0.

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

        Raises:
            KeyError: If an invalid keyword is found.

        """
        super(UDPChannel, self).__init__(ctx)

        local = kwargs.pop('local',None)

        local_port = kwargs.pop('local_port',None)

        self._tos = kwargs.pop('tos',0)

        self.__on_message = kwargs.pop('on_message',lambda *args : None)

        if local == None:
            raise KeyError('{} missing kwarg: local'.format(self.__class__.__name__))

        if local_port == None:
            raise KeyError('{} missing kwarg: local_port'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))

        self.__addr_info = socket.getaddrinfo(local,local_port,0,0,socket.SOL_UDP)

    def start(self):
        """Starts the channel.

        Raises:
            RuntimeError: If an socket or socket option error occurs.

        """
        af_type = self.__addr_info[0][0]

        self.__socket = socket.socket(self.__addr_info[0][0],socket.SOCK_DGRAM,0)

        self.__socket.bind(self.__addr_info[0][4])

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
        """
        Sends a message over the channel.

        Args:
          data (str): Data to send.

        Keyword Args:
          remote ((str,int)): Remote address and port.

        Rasies:
          KeyError:  If an invalid or missing keyword is found.

        """
        remote = kwargs.pop('remote',None)

        if remote == None:
            raise KeyError('{} missing kwarg: remote'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))

        self.__socket.sendto(data,remote)

    def destroy(self):
        """Destroys the channel.

        """
        self.ctx.remove_fd(self.__socket.fileno())
        self.__socket.close()
