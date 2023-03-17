# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
An example of use of a speaker device to perform text-to-speech.
"""

from controller import Robot
import os


class Controller(Robot):
    timeStep = 64

    def __init__(self):
        super(Controller, self).__init__()
        self.speaker = self.getDevice('speaker')

    def run(self):
        if os.name == 'nt':
            self.speaker.setEngine('microsoft')
        self.speaker.setLanguage('en-US')
        # use the speaker to perform text-to-speech
        self.speaker.speak('Hello! Using the text-to-speech of the Speaker device, I can speak 6 different languages: ' +
                           'English with US or UK accent, German, Spanish, French and Italian. ' +
                           'Using tags I can modulate my speech, like for example change ' +
                           '<prosody pitch="+16.8st">the pitch of my voice</prosody>, ' +
                           '<prosody pitch="-15st">and speak with a very low pitch</prosody>. ' +
                           '<prosody rate="0.5">And I can change the speed</prosody>' +
                           '<prosody rate="1.5">at which I speak</prosody>. ' +
                           'I can also <prosody volume="20">adjust the volume of my voice</prosody>. ' +
                           'Last but not least, I can imitate animals: <audio src="sounds/cow.wav">Meuh</audio>',
                           1.0)

        # empty control loop
        while self.step(self.timeStep) != -1:
            pass


controller = Controller()
controller.run()
