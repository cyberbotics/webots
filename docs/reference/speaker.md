## Speaker

Derived from [Device](device.md) and [Solid](solid.md).

```
Speaker {
}
```

### Description

The [Speaker](#speaker) node represents a loudspeaker device that can be embedded onboard a robot or standing in the environment.
It can be used to play sounds and perform text-to-speech from the controller API.

### Speaker Functions

#### `wb_speaker_play_sound`

%tab-component "language"

%tab "C"

```c
#include <webots/speaker.h>

void wb_speaker_play_sound(WbDeviceTag left, WbDeviceTag right, const char *sound, double volume, double pitch, double balance, bool loop);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Speaker.hpp>

namespace webots {
  class Speaker : public Device {
    static void playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance, bool loop);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Speaker

class Speaker (Device):
    @staticmethod
    def playSound(left, right, sound, volume, pitch, balance, loop):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Speaker;

public class Speaker extends Device {
  public static void playSound(Speaker left, Speaker right, String sound, double volume, double pitch, double balance, boolean loop);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_speaker_play_sound(left, right, sound, volume, pitch, balance, loop)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/play_sound` | `service` | `webots_ros::speaker_play_sound` | `string sound`<br/>`float64 volume`<br/>`float64 pitch`<br/>`float64 balance`<br/>`int8 loop`<br/>`---`<br/>`int8 success` |

%tab-end

%end

##### Description

*plays a sound*

This function allows the user to play a WAV sound file.

The function takes as arguments two speaker `WbDeviceTag` respectively for the left and right channels.
If both channels should be played on the same speaker or the file has only one channel, it is possible to pass the same device tag for both left and right arguments.
Alternatively, if one channel should be ignored, it is possible to pass `0` instead of one of the two tags.

The `sound` argument specifies the path to the sound file that should be played.
The `volume` argument allows the user to specify the volume of this sound (between 0.0 and 1.0).
The `pitch` argument allows the user to modify the pitch of the sound, the default sound pitch is multiplied by the pitch argument.
The `pitch` argument should be positive.
A value of 1.0 means no pitch change.
The `balance` argument allows the user to specify the balance between the left and the right speaker (between -1.0 and 1.0).
A value of 0 means no balance: both channels have the same volume.
A value of -1.0 means that the right channel is muted.
A value of 1.0 means that the left channel is muted.
Intermediate values define a difference of volume between the left and right channels.
Finally, the boolean `loop` argument defines if the sound will be played only once or repeatedly.

It is possible to change the volume, pitch, balance, and loop parameters of a sound currently playing by calling again the `wb_speaker_play_sound` function with the same speakers and `sound` arguments.

> **Note**: The path to the sound file should be defined either absolutely or relatively.
If defined relatively, it will be searched first relatively to the robot controller folder.
If not found there and if the robot is a PROTO, it will be searched relatively to the PROTO folder of the robot.

---

#### `wb_speaker_stop`

%tab-component "language"

%tab "C"

```c
#include <webots/speaker.h>

void wb_speaker_stop(WbDeviceTag tag, const char *sound);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Speaker.hpp>

namespace webots {
  class Speaker : public Device {
    void stop(const std::string &sound);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Speaker

class Speaker (Device):
    def stop(self, sound):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Speaker;

public class Speaker extends Device {
  public void stop(String sound);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_speaker_stop(tag, sound)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/stop` | `service` | [`webots_ros::set_string`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*stops the speaker*

This function stops a specific sound.
The `sound` argument is the path to an audio file currently playing in the speaker.
It should be the same path as the one previously provided to the `wb_speaker_play_sound` function.

It is possible to stop all the sounds currently playing in a speaker by setting `sound` to `NULL`.

---

#### `wb_speaker_is_sound_playing`

%tab-component "language"

%tab "C"

```c
#include <webots/speaker.h>

bool wb_speaker_is_sound_playing(WbDeviceTag tag, const char *sound);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Speaker.hpp>

namespace webots {
  class Speaker : public Device {
    bool isSoundPlaying(const std::string &sound) const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Speaker

class Speaker (Device):
    def isSoundPlaying(self, sound):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Speaker;

public class Speaker extends Device {
  public boolean isSoundPlaying(String sound);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_speaker_is_sound_playing(tag, sound)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/is_sound_playing` | `service` | `webots_ros::speaker_is_sound_playing` | `string sound`<br/>`---`<br/>`bool value`<br/> |

%tab-end

%end

##### Description

*check if the sound is been played*

This function can be used to check if the given sound is currently playing.
It can be used for example to check when a sound is over.
It is possible to check if the speaker is playing any sound (including text-to-speech) by setting `sound` to `NULL` (or to an empty string in object-oriented APIs).

---

#### `wb_speaker_set_engine`
#### `wb_speaker_set_language`
#### `wb_speaker_get_engine`
#### `wb_speaker_get_language`
#### `wb_speaker_is_speaking`
#### `wb_speaker_speak`

%tab-component "language"

%tab "C"

```c
#include <webots/speaker.h>

bool wb_speaker_set_engine(WbDeviceTag tag, const char *engine);
bool wb_speaker_set_language(WbDeviceTag tag, const char *language);
const char *wb_speaker_get_engine(WbDeviceTag tag);
const char *wb_speaker_get_language(WbDeviceTag tag);
bool wb_speaker_is_speaking(WbDeviceTag tag);
void wb_speaker_speak(WbDeviceTag tag, const char *text, double volume);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Speaker.hpp>

namespace webots {
  class Speaker : public Device {
    std::string getEngine();
    std::string getLanguage();
    bool setEngine(const std::string &engine);
    bool setLanguage(const std::string &language);
    bool isSpeaking() const;
    void speak(const std::string &text, double volume);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Speaker

class Speaker (Device):
    def getEngine(self):
    def getLanguage(self):
    def setEngine(self, engine):
    def setLanguage(self, language):
    def isSpeaking(self):
    def speak(self, text, volume):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Speaker;

public class Speaker extends Device {
  public std::string getLanguage();
  public std::string getEngine();
  public boolean setEngine(String engine);
  public boolean setLanguage(String language);
  public boolean isSpeaking();
  public void speak(String text, double volume);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
engine = wb_speaker_get_engine(tag)
language = wb_speaker_get_language(tag)
success = wb_speaker_set_engine(tag, engine)
success = wb_speaker_set_language(tag, language)
wb_speaker_is_speaking()
wb_speaker_speak(tag, text, volume)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_engine` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |
| `/<device_name>/get_language` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |
| `/<device_name>/set_engine` | `service` | [`webots_ros::set_string`](ros-api.md#common-services) | |
| `/<device_name>/set_language` | `service` | [`webots_ros::set_string`](ros-api.md#common-services) | |
| `/<device_name>/is_speaking` | `service` | [`webots_ros::get_bool`](ros-api.md#common-services) | |
| `/<device_name>/speak` | `service` | `webots_ros::speaker\speak` | `string text`<br/>`float64 volume`<br/>`---`<br/>`int8 success`<br/> |

%tab-end

%end

##### Description

*perform text-to-speech*

The `wb_speaker_set_engine` function allows the user to set the text-to-speech engine that is going to be used by a speaker.
The `engine` parameter should be one of the following values:

 - `"pico"` for the SVOX Pico text-to-speech engine (default value).
 - `"microsoft"` for the Microsoft SAPI5 text-to-speech engine (only available on Windows).

The function returns `false` if the engine cannot be set and `true` otherwise.

The `wb_speaker_get_engine` function allows the user to get the text-to-speech engine for a speaker device.

The `wb_speaker_set_language` function allows the user to set the language of the current text-to-speech engine.
For the `"pico"` engine, the `language` parameter should be set to one of the following values:

 - `"en-US"` for American English (default value).
 - `"en-UK"` for British English.
 - `"de-DE"` for German.
 - `"es-ES"` for Spanish.
 - `"fr-FR"` for French.
 - `"it-IT"` for Italian.

For the `"microsoft"` engine, it should follow the same format and correspond to an existing language, installed on the Windows computer.
The format is `"ll-CC"` where `ll` (lowercase) corresponds to an ISO 639-1 language code and `CC` (uppercase) corresponds to an ISO 3166 country code, like for example `"en-US"` or `"fr-FR"`.

The function will return `true` on success and `false` if it failed to set the requested `language` for the current engine.

The `wb_speaker_get_language` function allows the user to get the language of the text-to-speech for a speaker device.

The `wb_speaker_is_speaking` function will return `true` if the speaker is currently performing text-to-speech.
It can for example be used to check that the previous sentence is over before starting a new sentence.

The `wb_speaker_speak` function allows the user to execute text-to-speech on the speaker.
The value of the `text` parameter is converted into sound by Webots using the engine specified by the `wb_speaker_set_engine` function and the language specified by the `wb_speaker_set_language` function.
The resulting sound is played through the specified `speaker`.
The specified text could be plain text including punctuation signs such as "Hello world!", or can be enriched with special effects to make it more realistic.
Such effects are specified with XML tags compliant with the SSML (Speech Synthesis Markup Language) standard.
Here is a list of SSML tags that are supported by both the `pico` and the `microsoft` engines.
Additional tags and parameters may be supported by the `microsoft` engine.
Please refer to the [Microsoft Speech API (SAPI)](https://msdn.microsoft.com/en-us/library/ee125663.aspx) documentation about it.

**SSML Text-to-speech XML tags supported by the `pico` and `microsoft` engines**

- `prosody` has three supported parameters: `pitch`, `rate` and `volume`:

    - `pitch` is a relative value expressed as a number preceded by `+` or `-` and followed by `st`, that specifies an amount to change the pitch.
    For example `-2st`.
    The `st` suffix indicates the change unit is semitone, which is half of a tone (a half step) on the standard diatonic scale.

    - `rate` indicates the speaking rate (speed) of the contained text.
    This is a relative value, expressed as a number that acts as a multiplier of the default.
    For example, a value of `1` results in no change in the rate.
    A value of `.5` results in a halving of the rate.
    A value of `3` results in a tripling of the rate.

    - `volume` indicates the volume level of the speaking voice.
    This value should be expressed as a number in the range of `0` to `100`, from quietest to loudest.
    For example, `75`.
    The default is `100`.

- `audio` has one supported parameter which is `src`, specifying a WAV file.
  This results in the insertion of the specified sound file in the synthesized signal at the place specified in the input text.

Example:

```xml
Hello!
Using the text-to-speech of the Speaker device, I can speak 6 different languages: English with US or UK accent, German, Spanish, French and Italian.
Using tags I can modulate my speech, like for example change <prosody pitch="+16.8st">the pitch of my voice</prosody>, <prosody pitch="-15st">and speak with a very low pitch</prosody>.
<prosody rate="0.5">And I can change the speed</prosody><prosody rate="1.5">at which I speak</prosody>.
I can also <prosody volume="20">adjust the volume of my voice</prosody>.
Last but not least, I can imitate animals: <audio src="sounds/cow.wav">Meuh</audio>
```
