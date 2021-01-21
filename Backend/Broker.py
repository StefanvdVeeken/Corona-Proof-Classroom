#!/usr/bin/env python3
from pymongo import MongoClient
from json import JSONEncoder
import paho.mqtt.client as mqtt
import base64
import json
import codecs

class Chair:
    def __init__(self, id, neighbour_id, rssi):
        self._id = id
        self.neighbour_id = neighbour_id
        self.rssi = rssi
    _id = None
    neighbour_id = None
    rssi = None

class Window:
    def __init__(self, id, status):
        self._id = id
        self.status = status
    _id = None
    status = None
    
#MongoDB
db_client = MongoClient('localhost', 27017) #default parameters
db = db_client.sensors
windows = db['windows_collection']
chairs = db['chairs_collection']

# TheThingsNetwork
client_id = "LowPowerIotClientId"
broker = "eu.thethings.network"
appName = "corona_proof_classroom"
appID = "ttn-account-v2._MKGX66ysWDAsAyt9x78jT0wLfV_SyUjXjjptqYqOds"
sensorTypePosition = 0
dataPosition = 2

# ThingsBoard
thingsClientId = "thingsClient"
thingsBroker = 'thingsboard.cloud'
access_token = 'dspE60A3Lpf5iVrZSEaZ'
boardSubject = 'v1/gateway/telemetry'

def on_connect(client, userdata, flags, rc):
    print("Connected flags ",str(flags),"result code ",str(rc))

def on_message(client, userdata, msg):
    msg = json.loads(msg.payload)
    payloadRaw = msg["payload_raw"]
    payloadPlain = base64.b64decode(payloadRaw).hex()
    print(payloadPlain)
    parseData(payloadPlain)

def on_subscribe(client,userdata, mid, granted_qos):
	print ("Subscribed: "+ str(mid)+ " with QoS: " + str(granted_qos))

def parseData(sensorData):
    seperateMessages = [sensorData[i:i+8] for i in range(sensorTypePosition, len(sensorData), 8)] # Seperate all the messages
    for message in seperateMessages: # Now loop over all message and retrieve data
        dataType = message[sensorTypePosition:dataPosition] # Retrieve type of data from payload
        dataPieces = [message[i:i+2] for i in range(dataPosition, 8, 2)] # Get all data points. payload should only contain 4 bytes. In hex so 8 positions
        if dataType == format(ord('T'), 'x'): # Format hex value of T with 0x prefix
            uRSSI = int(dataPieces[2], 16)
            realRSSI = (-1)*(255+1-uRSSI) # Convert 2's complement to negative number. 255 for 8 bit number https://gist.github.com/ariutti/80173ac6fa1acef4c7ab2a9aecf038a5
            newChair = Chair(int(dataPieces[0], 16), int(dataPieces[1], 16), realRSSI)
            if chairs.count_documents ({'_id': newChair._id}, limit=1):
                # Update existing document
                chairs.update_one({"_id": newChair._id},
                    {"$set": { "neighbour_id": newChair.neighbour_id, "rssi": newChair.rssi}}            
                )
                tag = "Thingy " + str(newChair._id)
                newPayload = {
                    tag : [
                        {
                            "id": newChair._id,
                            "neighbour_id": newChair.neighbour_id,
                            "rssi": newChair.rssi
                        }
                    ]
                } # Prepare payload for Thingsboard
                message = thingsboard_client.publish(boardSubject, json.dumps(newPayload)) # Publish to Thingsboard
                message.wait_for_publish() # Wait for publish to Thingsboard to succeed
            else:
                # Insert new document
                chairs.insert_one(eval(json.dumps(newChair.__dict__)))
                tag = "Thingy " + str(newChair._id)
                newPayload = {
                    tag : [
                        {
                            "id": newChair._id,
                            "neighbour_id": newChair.neighbour_id,
                            "rssi": newChair.rssi
                        }
                    ]
                }
                message = thingsboard_client.publish(boardSubject, json.dumps(newPayload))
                message.wait_for_publish()
        elif dataType == format(ord('W'), 'x'):
            newWindow = Window(int(dataPieces[0], 16), int(dataPieces[1], 16))
            if windows.count_documents ({'_id': newWindow._id}, limit=1):
                # Update existing document
                windows.update_one({"_id": newWindow._id},
                    {"$set": { "status": newWindow.status}}            
                )
                tag = "Window " + str(newWindow._id)
                newPayload = {
                    tag : [
                        {
                            "id": newWindow._id,
                            "open": newWindow.status
                        }
                    ]
                }
                message = thingsboard_client.publish(boardSubject, json.dumps(newPayload))
                message.wait_for_publish()
            else:
                # Insert new document
                windows.insert_one(eval(json.dumps(newWindow.__dict__)))
                tag = "Window " + str(newWindow._id)
                newPayload = {
                    tag : [
                        {
                            "id": newWindow._id,
                            "open": newWindow.status
                        }
                    ]
                }
                message = thingsboard_client.publish(boardSubject, json.dumps(newPayload))
                message.wait_for_publish()


mqtt_client = mqtt.Client(client_id = client_id , clean_session = True, userdata = None, protocol = mqtt.MQTTv31)
mqtt_client.username_pw_set(appName, appID)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.on_subscribe = on_subscribe
mqtt_client.connect(broker, 1883)
mqtt_client.subscribe("+/devices/+/up")

thingsboard_client = mqtt.Client(client_id = thingsClientId, clean_session= True)
thingsboard_client.username_pw_set(access_token)
thingsboard_client.on_connect = on_connect
thingsboard_client.connect(thingsBroker, 1883)
thingsboard_client.loop_start()

while True:
    try:
        mqtt_client.loop_start()
        thingsboard_client.loop_start()
    except KeyboardInterrupt:
        break
    except Exception:
        break
mqtt_client.loop_stop()
thingsboard_client.loop_stop()
mqtt_client.disconnect()
thingsboard_client.disconnect()