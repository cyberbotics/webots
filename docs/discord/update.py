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

import argparse
import discord
import re

channels = [
    'news',
    'technical-questions',
    'development',
    'documentation'
]

contributors = {}


class MyClient(discord.Client):
    async def export_channel(self, channel):
        year = None
        with open(channel.name + '.md', 'w') as file:
            file.write('# %s\n\n' % channel.name.title())
            file.write('This is an archive of the `%s` channel of the ' % channel.name +
                       '[Webots Discord server](https://discordapp.com/invite/nTWbN9m).\n\n')
            async for message in channel.history(limit=100):
                if message.type == discord.MessageType.default and message.content:
                    # statistics
                    if message.author.name not in contributors:
                        contributors[message.author.name] = 0
                    else:
                        contributors[message.author.name] += 1
                    # yearly section
                    if year is None or year != message.created_at.year:
                        year = message.created_at.year
                        file.write('## %d\n\n' % year)
                    # author + date header
                    file.write('##### ' + message.author.name + ' ' + message.created_at.strftime("%m/%d/%Y %H:%M:%S") + '\n')
                    content = ''
                    # read message line by line
                    for line in message.content.splitlines():
                        # remove problematic parts
                        line = line.replace('<i>', '`<i>`')
                        # make url links
                        for url in re.findall(r'(?P<url>https?://[^\s]+)', line):
                            line = line.replace(url, '[%s](%s)' % (url, url))
                        # add line to the content
                        content += line + '\n'
                        # if quote add a new line to make distinction between message and quote
                        if line.startswith('> '):
                            content += '\n'
                    # remove last new line
                    content = content[:-1]
                    # replace mention by actual name
                    for mention in message.mentions:
                        alternativeMention = mention.mention.replace('<@', '<@!')
                        content = content.replace(alternativeMention, '`@' + mention.name + '`')
                        content = content.replace(mention.mention, '`@' + mention.name + '`')
                    file.write(content)
                    # add attachments
                    for attachment in message.attachments:
                        if attachment.filename.endswith('.png') or attachment.filename.endswith('.jpg'):
                            file.write('\n%figure\n')
                            file.write('![%s](%s)\n' % (attachment.filename, attachment.url))
                            file.write('%end\n')
                        else:
                            print("\033[33mUnsupported attachment file:" + attachment.filename + '\033[0m')
                    file.write('\n\n')
                elif message.type == discord.MessageType.pins_add:
                    pass
                else:
                    print("\033[33mUnsupported message type:" + str(message.type) + '\033[0m')
                    print("\033[33m\tContent:" + str(message.content) + '\033[0m')

    async def on_ready(self):
        with open('index.md', 'w') as file:
            file.write('# Webots Discord Archives\n\n')
            file.write('Release {{ webots.version.full }}\n\n')
            file.write('%figure\n')
            file.write('![Discord](images/discord.jpg)\n')
            file.write('%end\n\n')
            file.write('Copyright &copy; {{ date.year }} Cyberbotics Ltd.\n\n')
            file.write('These are archives of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m):\n')
            with open('menu.md', 'w') as menuFile:
                for channel in self.get_all_channels():
                    if type(channel) == discord.channel.TextChannel and channel.name in channels:
                        file.write('- [%s](%s)\n' % (channel.name.title(), channel.name + '.md'))
                        menuFile.write('- [%s](%s)\n' % (channel.name.title(), channel.name + '.md'))
                        await self.export_channel(channel)
            await self.close()

    async def on_message(self, message):
        print('Message from {0.author}: {0.content}'.format(message))


parser = argparse.ArgumentParser(description='Update the Webots discord doc.')
parser.add_argument('--token', '-t', dest='token', help='Specifies the Discord token', required=True)
parser.add_argument('--channels', '-c', dest='channels', nargs='+', help='list of channel to export')
args = parser.parse_args()
client = MyClient()
if args.channels is not None:
    channels = args.channels
client.run(args.token)

# display statistics
contributors = sorted(contributors.items(), key=lambda kv: kv[1])
contributors.reverse()
print('Top 20 contributors:')
for contributor in contributors[0:20]:
    print('  - %s (%d)' % contributor)
