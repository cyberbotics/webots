#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import discord
import sys

channels = [
    'news',
    #'technical-questions',
    #'development',
    #'documentation'
]


class MyClient(discord.Client):
    async def on_ready(self):
        print(self.cached_messages)
        print(len(self.cached_messages))
        for message in self.cached_messages:
            print(message)
        print(self.get_all_channels())
        for channel in self.get_all_channels():
            print(channel)
            if type(channel) == discord.channel.TextChannel and channel.name in channels:
                with open(channel.name + '.md', 'w') as file:  # TODO: slugify
                    async for message in channel.history(limit=20):
                        if message.type == discord.MessageType.default and message.content:
                            file.write('### ' + message.author.name + ' ' + message.created_at.strftime("%m/%d/%Y %H:%M:%S") + '\n')
                            file.write(message.content)
                            file.write('\n\n')
                            print([message.author.name, message.content, message.created_at, message.jump_url])
                        else:
                            print("Unsupported message type:" + str(message.type))
        await self.close()

    async def on_message(self, message):
        print('Message from {0.author}: {0.content}'.format(message))


client = MyClient()
client.run('MY_KEY')
