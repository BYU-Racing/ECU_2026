# can_serde.cpp
This module is responsible for serialization/deserialization of all the different CAN messages that we use in this project. What does that mean? Let me show you an example:

When you first receive a message from the CAN bus, it comes in a form that's not very usable. For example, when we receive the tire RPM message, it comes in like this:
```
| byte 0 | byte 1 | byte 2 | byte 3 | byte 4 | byte 5 | byte 6 | byte 7 |
|  0x01  |  0x00  |  0x00  |  0x00  |  0xB0  |  0x80  |  0x00  |  0x00  |
```
This by itself is really not very useful. So, we have to convert from the binary representation of the message to the usable form of the message. This process is called deserialization. The first thing we'll do is split the bytes up into groups, based on their purpose. The docs tell us that the zeroth byte is the tire position, and bytes 1-4 contain a float number. 

After we've split 