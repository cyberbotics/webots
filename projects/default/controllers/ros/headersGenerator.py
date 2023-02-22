#!/usr/bin/env python

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

import hashlib
try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x


class HeadersGenerator:
    messageTags = {
        'messageDefaultValue': '%defaultvalue%',
        'messageInitialization': '%init%',
        'messageTypeDefinition': '%typedef%',
        'messageDefinitionPrint': '%printdef%',
        'messageStream': '%stream%',
        'messageValuePrint': '%value%',
        'fixedSizeMessage': '%fixedsize%',
        'messageRequiredHeader': '%addedheaders%',
    }

    # these md5 sums are copied from the corresponding header files in 'include'
    # directory
    predefinedMD5 = {
        'Header': '2176decaecbce78abc3b96ef049fabed',
        'geometry_msgs/Point': '4a842b65f413084dc2b10fb484ea7f17',
        'geometry_msgs/PointStamped': 'c63aecb41bfdfd6b7e1fac37c7cbe7bf',
        'geometry_msgs/Quaternion': 'a779879fadf0160734f906b8c19c7004',
        'geometry_msgs/Transform': 'ac9eff44abf714214112b05d54a3cf9b',
        'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
        'geometry_msgs/TwistStamped': '98d34b0043a2093cf9d9345ab6eef12e',
        'geometry_msgs/Vector3': '4a842b65f413084dc2b10fb484ea7f17',
        'geometry_msgs/WrenchStamped': 'd78d3cb249ce23087ade7e7d0c40cfa7',
        'std_msgs/ColorRGBA': 'a29a96539573343b1310c73607334b00',
        'sensor_msgs/Illuminance': '8cf5febb0952fca9d650c3d11a81a188',
        'sensor_msgs/Image': '060021388200f6f0f447d0fcd9c64743',
        'sensor_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',
        'sensor_msgs/MagneticField': '2f3b0b43eed0c9501de0fa3ff89a45aa',
        'sensor_msgs/NavSatFix': '2d3a8cd499b9b4a0249fb98fd05cfa48',
        'sensor_msgs/PointCloud': 'd8e9c3f5afbdd8a130fd1d2763945fca',
        'sensor_msgs/Range': 'c005c34273dc426c67a020a87bc24148',
        'webots_ros/ContactPoint': 'c401f69a1503004a9e4aec8ae5ec3e17',
    }
    customPredefinedMD5 = {
        'webots_ros/msg/RecognitionObject.msg': 'd1a091cfdf9ce6628a657e03f119442a',
        'webots_ros/msg/RecognitionObjects.msg': 'ac0ec54e563936d28b7dec5cf26184c3',
    }

    def ros_md5sum(self, srv):
        for customMsg in self.customPredefinedMD5:
            if customMsg in srv:
                return self.customPredefinedMD5[customMsg]

        m = hashlib.md5()
        with open(srv, 'r') as f:
            text = f.read()
            text_in = StringIO()
            text_out = StringIO()
            accum = text_in
            for line in text.split('\n'):
                line = line.split('#')[0].strip()  # strip comments
                if "geometry_msgs/" in line or "std_msgs/" in line or "sensor_msgs/" in line or "Header" in line \
                   or 'webots_ros/' in line:
                    message = line.split(' ')[0]
                    # replace message by its corresponding MD5
                    # the fact that it is an array is not relevant for the computation of the MD5
                    clean_message = message.replace("[]", "")
                    if clean_message in self.predefinedMD5:
                        line = line.replace(message, self.predefinedMD5[clean_message])
                    else:
                        print('Error: undefined MD5 for: ' + message)
                if line.startswith('---'):  # lenient, by request
                    accum = text_out
                else:
                    accum.write(line + '\n')
        m.update(text_in.getvalue().encode('utf-8').strip())
        m.update(text_out.getvalue().encode('utf-8').strip())
        return m.hexdigest()

    def replace_template_tags(self, fnin, fnout, serviceName, file):
        with open(fnin, 'r') as fin:
            content = fin.read()
            content = content.replace('%SERVICE_NAME%', serviceName.upper())
            content = content.replace('%service_name%', serviceName)
            content = content.replace('%classname%', serviceName)
            content = content.replace('%md5sum%', self.ros_md5sum(file))
            with open(fnout, 'w') as fout:
                fout.write(content)

    def replace_message_tags(self, fnin, fnout, messageNames, messageTypes):
        with open(fnin, 'r') as fin:
            with open(fnout, 'w') as fout:
                for line in fin:
                    self.replace_tag_in_line(line, fout, messageNames, messageTypes)

    def replace_tag_in_line(self, line, fout, messageNames, messageTypes):
        tagReplacementString = ""
        messageNumber = len(messageTypes) - 1
        if line.find(self.messageTags['messageDefaultValue']) != -1:
            while messageNumber >= 0:
                type = messageTypes[messageNumber]
                if (type == 'uint8' or type == 'uint32' or type == 'uint64' or type == 'int8' or
                        type == 'int32' or type == 'int32[]'):
                    tagReplacementString += messageNames[messageNumber] + '(0)'
                elif type == 'float64' or type == 'float64[]' or type == 'float32' or type == 'float32[]':
                    tagReplacementString += messageNames[messageNumber] + '(0.0)'
                elif type == 'bool':
                    tagReplacementString += messageNames[messageNumber] + '(false)'
                elif type == 'string' or type == 'char[]' or type == 'string[]' or type == 'Header':
                    tagReplacementString += messageNames[messageNumber] + '()'
                elif '/' in type:  # composed type
                    tagReplacementString += messageNames[messageNumber] + '()'
                else:
                    print('Error: unsupported message type:' + type)
                if messageNumber >= 1:
                    tagReplacementString += '\n    , '
                messageNumber -= 1
            line = line.replace(self.messageTags['messageDefaultValue'], tagReplacementString)

        elif line.find(self.messageTags['messageInitialization']) != -1:
            while messageNumber >= 0:
                type = messageTypes[messageNumber]
                if (type == 'uint8' or type == 'uint32' or type == 'uint64' or type == 'int8' or
                        type == 'int32' or type == 'int32[]'):
                    tagReplacementString += messageNames[messageNumber] + '(0)'
                elif type == 'float64' or type == 'float64[]' or type == 'float32' or type == 'float32[]':
                    tagReplacementString += messageNames[messageNumber] + '(0.0)'
                elif type == 'bool':
                    tagReplacementString += messageNames[messageNumber] + '(false)'
                elif type == 'string' or type == 'char[]' or type == 'string[]' or type == 'Header':
                    tagReplacementString += messageNames[messageNumber] + '(_alloc)'
                elif '/' in type:  # composed type
                    tagReplacementString += messageNames[messageNumber] + '(_alloc)'
                else:
                    print('Error: unsupported message type:' + type)
                if messageNumber >= 1:
                    tagReplacementString += '\n    , '
                messageNumber -= 1
            line = line.replace(self.messageTags['messageInitialization'], tagReplacementString)

        elif line.find(self.messageTags['messageTypeDefinition']) != -1:
            while messageNumber >= 0:
                tagReplacementString += '   typedef '
                type = messageTypes[messageNumber]
                if type == 'uint8' or type == 'uint32' or type == 'uint64' or type == 'int8' or type == 'int32':
                    tagReplacementString += type + '_t '
                elif type == 'bool':
                    tagReplacementString += 'uint8_t '
                elif type == 'float64':
                    tagReplacementString += 'double '
                elif type == 'float32':
                    tagReplacementString += 'float '
                elif type == 'string':
                    tagReplacementString += \
                        'std::basic_string<char, std::char_traits<char>, typename ' \
                        'ContainerAllocator::template rebind<char>::other >  '
                elif type == 'int32[]':
                    tagReplacementString +=  \
                        'std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other > '
                elif type == 'char[]':
                    tagReplacementString += \
                        'std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other > '
                elif type == 'float64[]':
                    tagReplacementString += 'std::vector<double, typename ContainerAllocator::template rebind<double>::other > '
                elif type == 'float32[]':
                    tagReplacementString += 'std::vector<float, typename ContainerAllocator::template rebind<double>::other > '
                elif type == 'string[]':
                    tagReplacementString += \
                        'std::vector<std::basic_string<char, std::char_traits<char>, typename' \
                        ' ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template' \
                        ' rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template' \
                        ' rebind<char>::other > >::other >  '
                elif type == 'Header':
                    tagReplacementString += '::std_msgs::Header_<ContainerAllocator> '
                elif '/' in type:  # composed type
                    if "[]" in type:  # The array should be replaced bye a std::vector
                        cleanType = type.replace("[]", "")
                        tagReplacementString += 'std::vector< ::' + cleanType.split('/')[0] + '::' + cleanType.split('/')[1]
                        tagReplacementString += '_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::'
                        tagReplacementString += cleanType.split('/')[0] + '::' + cleanType.split('/')[1]
                        tagReplacementString += '_<ContainerAllocator> >::other > '
                    else:
                        tagReplacementString += ' ::' + type.split('/')[0] + '::' + type.split('/')[1]
                        tagReplacementString += '_<ContainerAllocator> '
                else:
                    print('Error: unsupported message type:' + type)
                tagReplacementString += ' _' + messageNames[messageNumber] + '_type;\n'
                tagReplacementString += '  _' + messageNames[messageNumber] + '_type ' + messageNames[messageNumber] + ';\n\n'
                messageNumber -= 1
            line = line.replace(self.messageTags['messageTypeDefinition'], tagReplacementString)

        elif line.find(self.messageTags['fixedSizeMessage']) != -1:
            syzeType = True
            while messageNumber >= 0:
                type = messageTypes[messageNumber]
                if (type == 'string' or type == 'int32[]' or type == 'char[]' or type == 'float64[]' or type == 'float32[]' or
                        type == 'string[]' or type == 'Header' or type == 'std_msgs/Float32MultiArray' or
                        type == 'sensor_msgs/PointCloud'):
                    syzeType = False
                messageNumber -= 1
            if syzeType:
                line = line.replace(self.messageTags['fixedSizeMessage'], 'TrueType')
            else:
                line = line.replace(self.messageTags['fixedSizeMessage'], 'FalseType')

        elif line.find(self.messageTags['messageDefinitionPrint']) != -1:
            while messageNumber >= 0:
                tagReplacementString += messageTypes[messageNumber] + ' ' + messageNames[messageNumber] + '\\n\\'
                if messageNumber >= 1:
                    tagReplacementString += '\n'
                messageNumber -= 1
            line = line.replace(self.messageTags['messageDefinitionPrint'], tagReplacementString)

        elif line.find(self.messageTags['messageStream']) != -1:
            while messageNumber >= 0:
                tagReplacementString += '      stream.next(m.' + messageNames[messageNumber] + ');\n'
                messageNumber -= 1
            line = line.replace(self.messageTags['messageStream'], tagReplacementString)

        elif line.find(self.messageTags['messageValuePrint']) != -1:
            while messageNumber >= 0:
                type = messageTypes[messageNumber]
                if type == 'uint8' or type == 'uint32' or type == 'uint64' or type == 'int8' or type == 'int32':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    Printer<' + type + '_t>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + ');\n'
                elif type == 'bool':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    Printer<uint8_t>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + ');\n'
                elif type == 'float64':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    Printer<double>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + ');\n'
                elif type == 'float32':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    Printer<float>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + ');\n'
                elif type == 'string':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    Printer<std::basic_string<char, std::char_traits<char>, typename '
                    tagReplacementString += 'ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + ');\n'
                elif type == 'int32[]':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]: ";\n'
                    tagReplacementString += '    for (size_t i = 0; i < v.' + messageNames[messageNumber] + '.size(); ++i)\n'
                    tagReplacementString += '    {\n    	s << indent << "  ' + messageNames[messageNumber]
                    tagReplacementString += '[" << i << "]: ";\n'
                    tagReplacementString += '    	Printer<int32_t>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + '[i]);\n    }\n'
                elif type == 'char[]':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]: ";\n'
                    tagReplacementString += '    for (size_t i = 0; i < v.' + messageNames[messageNumber] + '.size(); ++i)\n'
                    tagReplacementString += '    {\n    	s << indent << "  ' + messageNames[messageNumber]
                    tagReplacementString += '[" << i << "]: ";\n'
                    tagReplacementString += '    	Printer<uint8_t>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + '[i]);\n    }\n'
                elif type == 'float64[]':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]: ";\n'
                    tagReplacementString += '    for (size_t i = 0; i < v.' + messageNames[messageNumber] + '.size(); ++i)\n'
                    tagReplacementString += '    {\n    	s << indent << "  ' + messageNames[messageNumber]
                    tagReplacementString += '[" << i << "]: ";\n'
                    tagReplacementString += '    	Printer<double>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + '[i]);\n    }\n'
                elif type == 'float32[]':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]: ";\n'
                    tagReplacementString += '    for (size_t i = 0; i < v.' + messageNames[messageNumber] + '.size(); ++i)\n'
                    tagReplacementString += '    {\n    	s << indent << "  ' + messageNames[messageNumber]
                    tagReplacementString += '[" << i << "]: ";\n'
                    tagReplacementString += '    	Printer<float>::stream(s, indent + "  ", v.'
                    tagReplacementString += messageNames[messageNumber] + '[i]);\n    }\n'
                elif type == 'string[]':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]: ";\n'
                    tagReplacementString += '    for (size_t i = 0; i < v.' + messageNames[messageNumber] + '.size(); ++i)\n'
                    tagReplacementString += '    {\n    	s << indent << "  ' + messageNames[messageNumber]
                    tagReplacementString += '[" << i << "]: ";\n'
                    tagReplacementString += '    	Printer<std::basic_string<char, std::char_traits<char>, '
                    tagReplacementString += 'typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent +'
                    tagReplacementString += ' "  ", v.' + messageNames[messageNumber] + '[i]);\n    }\n'
                elif type == 'Header':
                    tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                    tagReplacementString += '    s << std::endl;\n'
                    tagReplacementString += '    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "'
                    tagReplacementString += '  ", v.' + messageNames[messageNumber] + ');\n'
                elif '/' in type:  # composed type
                    if "[]" in type:  # The array should be replaced bye a std::vector
                        cleanType = type.replace("[]", "")
                        tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + '[]" << std::endl;\n'
                        tagReplacementString += '    for (size_t i = 0; i < v.'
                        tagReplacementString += messageNames[messageNumber] + '.size(); ++i)\n'
                        tagReplacementString += '    {\n    	s << indent << "  '
                        tagReplacementString += messageNames[messageNumber] + '[" << i << "]: ";\n'
                        tagReplacementString += '    	s << std::endl;\n'
                        tagReplacementString += '    	s << indent;\n'
                        tagReplacementString += '    	Printer< ::' + cleanType.split('/')[0] + '::' + cleanType.split('/')[1]
                        tagReplacementString += '_<ContainerAllocator> >::stream(s, indent + "    ", v.'
                        tagReplacementString += messageNames[messageNumber] + '[i]);\n    }\n'
                    else:
                        tagReplacementString += '    s << indent << "' + messageNames[messageNumber] + ': ";\n'
                        tagReplacementString += '    s << std::endl;\n'
                        tagReplacementString += '    Printer< ::' + type.split('/')[0] + '::' + type.split('/')[1]
                        tagReplacementString += '_<ContainerAllocator> >::stream(s, indent + "  ", v.'
                        tagReplacementString += messageNames[messageNumber] + ');\n'
                else:
                    print('Error: unsupported message type:' + type)
                messageNumber -= 1
            line = line.replace(self.messageTags['messageValuePrint'], tagReplacementString)
        elif line.find(self.messageTags['messageRequiredHeader']) != -1:
            if 'Header' in messageTypes:
                tagReplacementString += '#include <std_msgs/Header.h>\n'
            while messageNumber >= 0:
                type = messageTypes[messageNumber].replace("[]", "")  # the array should be omitted for the include
                if '/' in type:  # composed type
                    tagReplacementString += '#include <' + type + '.h>\n'
                messageNumber -= 1
            line = line.replace(self.messageTags['messageRequiredHeader'], tagReplacementString)
        fout.write(line)
