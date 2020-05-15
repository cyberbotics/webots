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

channels = [
    'news',
    'technical-questions',
    'development',
    'documentation'
]


class MyClient(discord.Client):
    async def export_channel(self, channel):
        year = None
        with open('reference/discord_' + channel.name + '.md', 'w') as file:
            file.write('# %s\n\n' % channel.name.title())
            file.write('This is an archive of the `%s` channel of the ' % channel.name +
                       '[Webots Discord server](https://discordapp.com/invite/nTWbN9m).\n\n')
            async for message in channel.history(limit=20):
                if message.type == discord.MessageType.default and message.content:
                    if year is None or year != message.created_at.year:
                        year = message.created_at.year
                        file.write('## %d\n\n' % year)
                    file.write('##### ' + message.author.name + ' ' + message.created_at.strftime("%m/%d/%Y %H:%M:%S") + '\n')
                    file.write(message.content)
                    print(message.attachments)
                    for attachment in message.attachments:
                        print([attachment.filename, filename.url])
                    file.write('\n\n')
                else:
                    print("Unsupported message type:" + str(message.type))
                    print("\tContent:" + str(message.content))

    async def on_ready(self):
        with open('reference/discord.md', 'w') as file:
            file.write('# Discord Archives\n')
            file.write('These are archives of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).\n\n')
            for channel in self.get_all_channels():
                if type(channel) == discord.channel.TextChannel and channel.name in channels:
                    file.write('- [%s](%s)\n' % (channel.name.title(), 'discord_' + channel.name + '.md'))
                    await self.export_channel(channel)
            await self.close()

    async def on_message(self, message):
        print('Message from {0.author}: {0.content}'.format(message))


client = MyClient()
client.run('NjY2NTQ5NTczNjgyMDY5NTQ1.Xr5-8A.WE85Ap174W6VdzbQX_kf8gzSDC8')
