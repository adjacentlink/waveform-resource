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

"""Channel interface realized by all derived communication
channels. Used internally to manage channel instances.

"""
from __future__ import absolute_import, division, print_function
import abc

class Channel(object):
    """
    Channel base class.
    """
    def __init__(self,ctx):
        """
        Creates a channel

        Args:
          ctx (obj): context instance.

        """
        self.__ctx = ctx
        self.__id = None

    @property
    def ctx(self):
        """
        (obj) Channel context.
        """
        return self.__ctx

    @property
    def id(self):
        """
        (int) Channel id.
        """
        return self.__id

    @id.setter
    def id(self,channel_id):
        self.__id = channel_id

    @abc.abstractmethod
    def start(self):
        """
        Starts the channel.
        """

    @abc.abstractmethod
    def read(self):
        """
        Reads data from the channel.
        """

    @abc.abstractmethod
    def send(self,data,**kwargs):
        """Writes data to the channel.

        Args:
          data (str): Data to write.

          kwargs: Derived channel specific keyword arguments.

        """

    @abc.abstractmethod
    def destroy(self):
        """
        Destroys the channel.
        """
