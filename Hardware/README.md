# BikeNode
## System Requirements
### 1. Wireless Communication (LoRa Ra-02)
Long range, reliable, bidirectional interference-proof point-to-point communication .
This necessitates: 
- a long range wireless transciever
	
### 2. Messaging
An abstraction layer that presents enough information to structure sent and received data as in instant messaging apps.
This necessitates:
- timestamping
- a node address scheme

### 3. Plug and Play PC compatibility
Pinout compatible with the most common USB-UART converters. 
This necessitates:
- command set to be used over UART

### 4. Security
User authentication before accessing/sending message. Secure message transmission
This necessitates:
- Encryption

## Feature summary
### Hardware
- [ ] LoRa setup
	- [ ] Schematic
- [ ] STM32F030F4P6 setup
	- [ ] Schematic

### Software
- [ ] Packet
	- [ ] Structure
	- [ ] Addressing scheme
- [ ] LoRa
	- [ ] Transmision/receive
	- [ ] Sleep mode
- [ ] Messaging
	- [ ] Encryption
	- [ ] Sender timestamping
	- [ ] Buffer messages to send when the receiever is online
	- [ ] Buffer received messages till software accesses them
- [ ] User Authentication
- [ ] MCU 
	- [ ] Sleep


## Hardware Implementation


## Software Implementation
