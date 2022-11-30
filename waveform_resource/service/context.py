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
"""Application context for creating communication channels and
timers.

"""
from __future__ import absolute_import, division, print_function
import select
from threading import RLock, Thread
import time
import ctypes
import struct
import os
import functools
import logging
import traceback
from sortedcontainers import SortedDict

from .udpchannel import UDPChannel
from .tcpchannel import TCPServerChannel,TCPClientChannel
from .multicastchannel import MulticastChannel

class Context(object):
    class ITimerSpec(ctypes.Structure):
            _fields_ = [
                ('it_interval_tv_sec',ctypes. c_long),
                ('it_interval_tv_nsec', ctypes.c_long),
                ('it_value_tv_sec', ctypes.c_long),
                ('it_value_tv_nsec', ctypes.c_long)
            ]

    def __init__(self):
        """
        Creates the application context.
        """
        self.__epoll = select.epoll()
        self.__fd_handler_map = {}
        self.__channel_map = {}
        self.__channel_sequence_id = 1
        self.__timer_map = SortedDict()
        self.__timer_sequence_map = SortedDict()
        self.__timer_sequence_id = 1
        self.__lock = RLock()
        self.__thread = None
        self.__libc = ctypes.CDLL(None)
        self.__eventfd = self.__libc.eventfd(ctypes.c_uint(0),ctypes.c_uint(0))
        self.__timerfd = self.__libc.timerfd_create(ctypes.c_int(0),ctypes.c_int(0))
        self.__epoll.register(self.__eventfd,select.EPOLLIN)
        self.__epoll.register(self.__timerfd,select.EPOLLIN)

    def __register_channel(self,channel):
        """Registers a channel and assign a channel id.

        Args:
          channel (obj): Channel instance.

        Returns:
          int: Channel id.

        """
        channel_id = None

        with self.__lock:
            channel_id = self.__channel_sequence_id
            self.__channel_map[channel_id] = channel
            self.__channel_sequence_id += 1

        channel.id = channel_id

        return channel_id

    def delete_channel(self,channel_id):
        """Deletes a channel and assign a channel id.

        Args:
          channel (obj): Channel instance.

        """
        with self.__lock:
            self.__channel_map[channel_id].destroy()
            del self.__channel_map[channel_id]

    def create_channel_udp(self,**kwargs):
        """Creates a UDP channel instance. Supports IPv4 and IPv6.

        Keyword Args:
          local (str): Local address or hostname.

          local_port (int): Local port.

          tos (int): IP tos value for transmitted packets. IPv4
            only. Default 0.

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

        Returns:
          int: Channel id.

        Raises:
            KeyError: If an invalid keyword is found.

        """
        channel = UDPChannel(self,**kwargs)

        channel_id = self.__register_channel(channel)

        channel.start()

        return channel_id

    def create_channel_tcp_server(self,**kwargs):
        """Creates a TCP server channel instance. Supports IPv4 and IPv6.

        Keyword Args:
          local (str): Listen address or hostname.

          local_port (int): Listen port.

          on_accept (callable): Callable object invoked on connection
            accept.  Callable passed 3 parameters: context (obj),
            channel id (int), and remote address and port tuple
            ((str,int)).

          on_message (callable): Callable object invoked on message
            receive. Callable passed 3 parameters: context (obj),
            channel id (int), and data (str).

          on_close (callable): Callable object invoked on connection
             close.  Callable passed 3 parameters: context (obj),
             channel id (int), and remote address and port tuple
             ((str,int)).

        Returns:
          int: Channel id.

        Raises:
            KeyError: If an invalid keyword is found.

        """
        channel = TCPServerChannel(self,**kwargs)

        channel_id = self.__register_channel(channel)

        channel.start()

        return channel_id

    def create_channel_tcp_client(self,**kwargs):
        """Creates a TCP client channel instance. Supports IPv4 and
        IPv6. Channel will reconnect on remote server disconnect.

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

        Returns:
          int: Channel id.

        Raises:
            KeyError: If an invalid keyword is found.

        """
        channel = TCPClientChannel(self,**kwargs)

        channel_id = self.__register_channel(channel)

        channel.start()

        return channel_id

    def create_channel_multicast(self,**kwargs):
        """Creates a Multicast channel instance. Supports IPv4 and IPv6.

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

        Returns:
          int: Channel id.

        Raises:
            KeyError: If an invalid keyword is found.

        """
        channel = MulticastChannel(self,**kwargs)

        channel_id = self.__register_channel(channel)

        channel.start()

        return channel_id

    def channel_send(self,channel_id,data,**kwargs):
        """Sends a message over the channel.

        Args:
          channel_id (int): Channel id to send.

          data (str): Data to send.

        Keyword Args:
          For UDP and TCP Server channels:

            remote ((str,int)): Remote address and port.

        Raises:
            ValueError: If an invalid channel id is specified.

        """
        with self.__lock:
            if channel_id in self.__channel_map:
                self.__channel_map[channel_id].send(data,**kwargs)
            else:
                raise ValueError('invalid channel id: {}'.format(channel_id))

    def add_fd(self,fileno,handler):
        """Adds a file descriptor to internal epoll processing.

        Args:
          fileno (int): File descriptor.

          handler (callable): Callable invoked when file descriptor
            input is available. Callable passed no arguments.

        """
        with self.__lock:
            self.__epoll.register(fileno,select.EPOLLIN)
            self.__fd_handler_map[fileno] = handler

    def remove_fd(self,fileno):
        """Removes a file descriptor to internal epoll processing.

        Args:
          fileno (int): File descriptor.

        """
        with self.__lock:
            if fileno in self.__fd_handler_map:
                self.__epoll.unregister(fileno)
                del self.__fd_handler_map[fileno]

    def create_timer(self,abs_time_expire,handler):
        """Creates a timer.

        Args:
          abs_time_expire (float): Absolute expire time in seconds.

          handler (callable): Callable object invoked on timer expire.
             Callable passed 2 parameters: context (obj) and timer id
             (int).

        """
        with self.__lock:
            abs_time_sec,abs_time_usec = int(abs_time_expire), (abs_time_expire % 1) * 1000000

            if len(self.__timer_map):
                if (abs_time_sec,abs_time_usec ) <  self.__timer_map.peekitem(index=0)[0]:
                    # clear exiting the timer
                    spec = Context.ITimerSpec()
                    spec.it_interval_tv_sec = ctypes.c_long(0)
                    spec.it_interval_tv_nsec = ctypes.c_long(0)
                    spec.it_value_tv_sec = ctypes.c_long(0)
                    spec.it_value_tv_nsec = ctypes.c_long(0)

                    self.__libc.timerfd_settime(ctypes.c_int(self.__timerfd),
                                                ctypes.c_int(1),
                                                ctypes.byref(spec),
                                                ctypes.c_void_p(0))

                    # schedule earlier timer
                    self.__timer_map[(abs_time_sec,abs_time_usec)] = [self.__timer_sequence_id]
                    self.__timer_sequence_map[self.__timer_sequence_id] = [abs_time_sec,abs_time_usec,handler]
                    self.__timer_sequence_id += 1

                    spec.it_interval_tv_sec = ctypes.c_long(0)
                    spec.it_interval_tv_nsec = ctypes.c_long(0)
                    spec.it_value_tv_sec = ctypes.c_long(abs_time_sec)
                    spec.it_value_tv_nsec = ctypes.c_long(int(abs_time_usec * 1000))

                    self.__libc.timerfd_settime(ctypes.c_int(self.__timerfd),
                                                ctypes.c_int(1),
                                                ctypes.byref(spec),
                                                ctypes.c_void_p(0))
                else:
                    # add the timer for later scheduling
                    if (abs_time_sec,abs_time_usec ) not in self.__timer_map:
                        self.__timer_map[(abs_time_sec,abs_time_usec )] = []

                    self.__timer_map[(abs_time_sec,abs_time_usec)].append(self.__timer_sequence_id)
                    self.__timer_sequence_map[self.__timer_sequence_id] = [abs_time_sec,abs_time_usec,handler]
                    self.__timer_sequence_id += 1

            else:
                # nothing in the timer map
                self.__timer_map[(abs_time_sec,abs_time_usec)] = [self.__timer_sequence_id]
                self.__timer_sequence_map[self.__timer_sequence_id] = [abs_time_sec,abs_time_usec,handler]
                self.__timer_sequence_id += 1

                spec = Context.ITimerSpec()
                spec.it_interval_tv_sec = ctypes.c_long(0)
                spec.it_interval_tv_nsec = ctypes.c_long(0)
                spec.it_value_tv_sec = ctypes.c_long(abs_time_sec)
                spec.it_value_tv_nsec = ctypes.c_long(int(abs_time_usec * 1000))

                self.__libc.timerfd_settime(ctypes.c_int(self.__timerfd),
                                            ctypes.c_int(1),
                                            ctypes.byref(spec),
                                            ctypes.c_void_p(0))


    def start(self):
        """Starts internal conext processing.

        """
        def procedure():
            cancel = False
            while not cancel:
                events = self.__epoll.poll()
                callables = []

                with self.__lock:
                    for fd,etype in events:
                        if fd == self.__eventfd:
                            cancel = True
                            break

                        elif fd == self.__timerfd:
                            data = os.read(self.__timerfd,8)

                            for abs_time_sec,abs_time_usec in self.__timer_map:
                                if(abs_time_sec + (abs_time_usec / 1000000.0) <= time.time()):
                                    for timer_sequence_id in self.__timer_map[(abs_time_sec,abs_time_usec)]:
                                        _,_,handler = self.__timer_sequence_map[timer_sequence_id]
                                        callables.append(functools.partial(handler,self,timer_sequence_id))
                                        del self.__timer_sequence_map[timer_sequence_id]

                                    del self.__timer_map[(abs_time_sec,abs_time_usec)]
                                else:
                                    break

                            # schedule next timer
                            if self.__timer_map:
                                spec = Context.ITimerSpec()
                                abs_time_sec,abs_time_usec = self.__timer_map.peekitem(index=0)[0]
                                spec.it_interval_tv_sec = ctypes.c_long(0)
                                spec.it_interval_tv_nsec = ctypes.c_long(0)
                                spec.it_value_tv_sec = ctypes.c_long(abs_time_sec)
                                spec.it_value_tv_nsec = ctypes.c_long(int(abs_time_usec * 1000))

                                self.__libc.timerfd_settime(ctypes.c_int(self.__timerfd),
                                                            ctypes.c_int(1),
                                                            ctypes.byref(spec),
                                                            ctypes.c_void_p(0))
                            pass

                        elif fd in self.__fd_handler_map:
                            callables.append(self.__fd_handler_map[fd])

                for action in callables:
                    try:
                        action()
                    except:
                        logging.error(traceback.format_exc())


        self.__thread = Thread(target=procedure)
        self.__thread.start()

    def stop(self):
        """Stops the application context.

        """
        if self.__thread != None:
            os.write(self.__eventfd,struct.pack('Q',1))
            self.__thread.join()
            self.__thread = None

    def destroy(self):
        """Destroys the application context.

        """
        with self.__lock:
            channels = list(self.__channel_map.values())

        for channel in channels:
            channel.destroy()
