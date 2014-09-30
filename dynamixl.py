#!/usr/bin/env python

def calcChecksum(Packet): #do not include checksum when calling

	checksum = 0x00

	for x in range(len(Packet)):
		if x>1:
			checksum = checksum + Packet[x]

	checksum = ~checksum
	checksum &= 0xFF
	Packet.append(checksum)
	return Packet
	
def movePacket(velocity, ID): #input ranges from +1 to -1, +forward/CW; -reverse/CCW
	#ID, length, instr, address1, address2, 
	instr = 0x20
	length = 0x02

	velocity = int(1023*velocity)
	
	if velocity < 0:
		direction = 0
	else:
		direction = 1

	magnatude = abs(velocity)
	magnatude &= 0x3FF
	packet = direction << 10
	packet |= magnatude
		
	
	
	byte1 = packet&0xFF00
	byte1 = byte1>>8	
	byte2 = packet&0xFF

	packet = calcChecksum([0XFF, 0XFF, ID, length, instr, byte2, byte1])

	return packet

