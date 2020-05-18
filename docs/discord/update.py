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
import os
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
        path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(path, channel.name + '.md'), 'w', encoding='utf-8') as file:
            file.write(u'# %s\n\n' % channel.name.title())
            file.write(u'This is an archive of the `%s` channel of the ' % channel.name +
                       '[Webots Discord server](https://discordapp.com/invite/nTWbN9m).\n\n')
            previousMessageUser = None
            async for message in channel.history(limit=None):
                if message.type == discord.MessageType.default and (message.content or message.attachments):
                    # statistics
                    if message.author.name not in contributors:
                        contributors[message.author.name] = 0
                    else:
                        contributors[message.author.name] += 1
                    # yearly section
                    if year is None or year != message.created_at.year:
                        year = message.created_at.year
                        file.write(u'## %d\n\n' % year)
                    # author + date header
                    if previousMessageUser != message.author:
                        previousMessageUser = message.author
                        roles = []
                        if hasattr(message.author, 'roles'):
                            for role in message.author.roles:
                                if role.name != '@everyone':
                                    roles.append(role.name)
                        file.write(u'##### %s %s%s\n' %
                                   (message.author.name,
                                    '[%s] ' % '-'.join(roles) if roles else '',
                                    message.created_at.strftime("%m/%d/%Y %H:%M:%S")))
                    else:
                        file.write('\n')
                    if message.content:
                        content = ''
                        # read message line by line
                        inCode = False
                        for line in message.content.splitlines():
                            # remove wrongly used multi-line code
                            for start, code, end in re.findall(r'([^`]*)```([^`]*)```([^`]*)', line):
                                line = '%s`%s`%s' % (start, code, end)
                            # multi-line code
                            if '```' in line:
                                inCode = not inCode
                                # make sure it is on a dedicated line
                                if inCode and not line.startswith('```'):
                                    line = line.replace('```', '\n```')
                                if not inCode and len(line) > 3:
                                    line = line.replace('```', '\n```')
                            # not inside a multi-line code
                            if not inCode:
                                # remove problematic parts
                                line = line.replace('<i>', '`<i>`')
                                if line.startswith('#'):
                                    print(line)  #TODO
                                # protect underscores
                                undescoreProtected = False
                                for start, code, end in re.findall(r'([^`]*)`([^`]*)`([^`]*)', line):
                                    line = line.replace(start, start.replace('_', '\\_'))
                                    line = line.replace(end, end.replace('_', '\\_'))
                                    undescoreProtected = True
                                if not undescoreProtected:
                                    line = line.replace('_', '\\_')
                                # make url links
                                for url in re.findall(r'(?P<url>https?://[^\s]+)', line):
                                    line = line.replace(url, '[%s](%s)' % (url, url.replace('\\_', '_')))
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
                        if (attachment.filename.endswith('.png') or
                                attachment.filename.endswith('.jpg') or
                                attachment.filename.endswith('.jpeg') or
                                attachment.filename.endswith('.gif') or
                                attachment.filename.endswith('.bmp') or
                                attachment.filename.endswith('.PNG') or
                                attachment.filename.endswith('.JPG') or
                                attachment.filename.endswith('.JPEG')):
                            file.write(u'\n%figure\n')
                            file.write(u'![%s](%s)\n' % (attachment.filename, attachment.url))
                            file.write(u'%end\n')
                        else:
                            file.write(u'\n> **Attachment**: [%s](%s)\n' % (attachment.filename.replace('_', '\\_'),
                                                                            attachment.url))
                    file.write(u'\n\n')
                elif message.type == discord.MessageType.pins_add or message.type == discord.MessageType.new_member:
                    pass
                else:
                    print("\033[33mUnsupported message type:" + str(message.type) + '\033[0m')
                    print("\033[33m\tContent:" + str(message.content) + '\033[0m')

    async def on_ready(self):
        path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(path, 'index.md'), 'w', encoding='utf-8') as file:
            file.write(u'# Webots Discord Archives\n\n')
            file.write(u'Release {{ webots.version.full }}\n\n')
            file.write(u'%figure\n')
            file.write(u'![Discord](images/discord.jpg)\n')
            file.write(u'%end\n\n')
            file.write(u'Copyright &copy; {{ date.year }} Cyberbotics Ltd.\n\n')
            file.write(u'These are archives of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m):\n')
            with open(os.path.join(path, 'menu.md'), 'w', encoding='utf-8') as menuFile:
                for channel in self.get_all_channels():
                    if type(channel) == discord.channel.TextChannel and channel.name in channels:
                        file.write(u'- [%s](%s)\n' % (channel.name.title(), channel.name + '.md'))
                        menuFile.write(u'- [%s](%s)\n' % (channel.name.title(), channel.name + '.md'))
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
