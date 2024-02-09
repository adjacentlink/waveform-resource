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

"""TCP Server and Client communication channels.

"""
from __future__ import absolute_import, division, print_function
from .channel import Channel
import socket
import functools
import time
import traceback
import logging

class TCPServerChannel(Channel):
    def __init__(self,ctx,**kwargs):
        """Creates the channel instance. Supports IPv4 and IPv6.

        Args:
          ctx (obj): Context instance.

        Keyword Args:
          local (str): Listen address or hostname.

          local_port (int): Listen port.

          on_accept (callable): Callable object invoked on connection
            accept.  Callable passed 4 parameters: context (obj),
            channel id (int), and remote address and port tuple
            ((str,int)) and the associated socket (socket)
            passed in kwargs as {'remote': socket}.

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

          on_close (callable): Callable object invoked on connection
             close.  Callable passed 3 parameters: context (obj),
             channel id (int), and remote address and port tuple
             ((str,int)).

        Raises:
            KeyError: If an invalid keyword is found.

        """
        super(TCPServerChannel, self).__init__(ctx)

        local = kwargs.pop('local',None)

        local_port = kwargs.pop('local_port',None)

        self.__on_accept = kwargs.pop('on_accept',lambda *args : None)

        self.__on_close = kwargs.pop('on_close',lambda *args : None)

        self.__on_message = kwargs.pop('on_message',lambda *args : None)

        self.__remotes = {}

        if local == None:
            raise KeyError('{} missing kwarg: local'.format(self.__class__.__name__))

        if local_port == None:
            raise KeyError('{} missing kwarg: local_port'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))


        self.__addr_info = socket.getaddrinfo(local,local_port,0,0,socket.SOL_TCP)

    def start(self):
        """Starts the channel.

        Raises:
            RuntimeError: If an socket or socket option error occurs.

        """
        self.__socket = socket.socket(self.__addr_info[0][0], socket.SOCK_STREAM,0)

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
        self.__socket.bind(self.__addr_info[0][4])
        self.__socket.listen(1)
        self.ctx.add_fd(self.__socket.fileno(),self.read)

    def read(self):
        """Reads from the channel and calls on_accept callable.

        Registers on_message callable for new client connection.

        """
        remote,address = self.__socket.accept()

        self.ctx.add_fd(remote.fileno(),
                        functools.partial(self.process_client,remote))

        self.__remotes[address] = remote

        try:
            self.__on_accept(self.ctx,self.id,address,**{'remote':remote})
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

          ValueError: If invalid remote is specified.

        """
        remote = kwargs.pop('remote',None)

        if remote == None:
            raise KeyError('{} missing kwarg: remote'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))
        if remote in self.__remotes:
            self.__remotes[remote].send(data)
        else:
            raise ValueError('{} unknown remote: {}'.format(self.__class__.__name__,
                                                            remote))

    def process_client(self,remote):
        """Process a client message calling on_message or on_close if a
        connection has closed.

        Args:
          remote (obj): Remote client socket.

        """
        data = remote.recv(65535)

        if len(data):
            try:
                self.__on_message(self.ctx,
                                  self.id,
                                  data,
                                  remote=remote.getpeername())
            except:
                logging.error(traceback.format_exc())
        else:
            self.ctx.remove_fd(remote.fileno())
            address = remote.getpeername()
            remote.close()
            del self.__remotes[address]
            try:
                self.__on_close(self.ctx,self.id,address)
            except:
                logging.error(traceback.format_exc())

    def destroy(self):
        """Destroys the channel.

        """
        self.ctx.remove_fd(self.__socket.fileno())
        self.__socket.close()


class TCPClientChannel(Channel):
    def __init__(self,ctx,**kwargs):
        """Creates the channel instance. Supports IPv4 and IPv6. Channel
        will reconnect on remote server disconnect.

        Args:
          ctx (obj): Context instance.

        Keyword Args:
          remote (str): Remote server address or hostname.

          remote_port (int): Remote port.

          on_connect (callable): Callable object invoked on connection.
            Callable passed 3 parameters: context (obj),
            channel id (int), and remote address and port tuple
            ((str,int)).

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

            Callable passed 1 keyword argument:

                remote ((str,int)): Remote address and port.

          on_close (callable): Callable object invoked on connection
             close.  Callable passed 3 parameters: context (obj),
             channel id (int), and remote address and port tuple
             ((str,int)).

        Raises:
            KeyError: If an invalid keyword is found.

        """
        super(TCPClientChannel, self).__init__(ctx)

        remote = kwargs.pop('remote',None)

        remote_port = kwargs.pop('remote_port',None)

        self.__on_connect = kwargs.pop('on_connect',lambda *args : None)

        self.__on_close = kwargs.pop('on_close',lambda *args : None)

        self.__on_message = kwargs.pop('on_message',lambda *args : None)

        if remote == None:
            raise KeyError('{} missing kwarg: remote'.format(self.__class__.__name__))

        if remote_port == None:
            raise KeyError('{} missing kwarg: remote_port'.format(self.__class__.__name__))

        if len(kwargs):
            raise KeyError('{} unknown kwarg(s): {}'.format(self.__class__.__name__,
                                                            ', '.join(list(kwargs.keys()))))

        addr_info = socket.getaddrinfo(remote,remote_port,0,0,socket.SOL_TCP)
        self.__remote = addr_info[0][4]
        self.__af_type = addr_info[0][0]
        self.__timer_sequence_id = 0

    def start(self):
        """Starts the channel.

        Raises:
            RuntimeError: If an socket or socket option error occurs.

        """
        self.__socket = socket.socket(self.__af_type,socket.SOCK_STREAM, 0)

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

        self.__connect(self.ctx)

    def read(self):
        """Reads from the channel and calls on_message or on_close callable.

        """
        data = self.__socket.recv(65535)

        if len(data):
            try:
                self.__on_message(self.ctx,self.id,data)
            except:
                logging.error(traceback.format_exc())
        else:
            self.ctx.remove_fd(self.__socket.fileno())
            self.__socket.close()
            try:
                self.__on_close(self.ctx,self.id,self.__remote)
            except:
                logging.error(traceback.format_exc())
            self.__socket = socket.socket(self.__af_type,socket.SOCK_STREAM, 0)
            self.__connect(self.ctx)

    def send(self,data,**kwargs):
        """Sends a message over the channel.

        Args:
          data (str): Data to send.

        """
        self.__socket.send(data)

    def destroy(self):
        """Destroys the channel.

        """
        self.ctx.remove_fd(self.__socket.fileno())
        self.__socket.close()

    def __connect(self,ctx,_=0):
        """Remote server reconnect timer callback.

        """
        try:
            self.__socket.connect(self.__remote)
            self.ctx.add_fd(self.__socket.fileno(),self.read)
            self.__timer_sequence_id = 0
            try:
                self.__on_connect(self.ctx,self.id,self.__remote)
            except:
                logging.error(traceback.format_exc())
        except:
            self.__timer_sequence_id = self.ctx.create_timer(time.time() + 1, self.__connect)
