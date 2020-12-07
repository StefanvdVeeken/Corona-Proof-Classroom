#!B:/School/Corona-Proof-Classroom/Backend/Scripts/python.exe
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

#Testdata
chair1 = Chair(1, 0, 0)
chair2 = Chair(2, 1, -100)
chair_list = [
    eval(json.dumps(chair1.__dict__)),
    eval(json.dumps(chair2.__dict__))
]
window_list = [
    eval(json.dumps(Window(1, 0).__dict__)),
    eval(json.dumps(Window(2, 1).__dict__))
]
if("windows_collection" in db.list_collection_names()):
    windows.drop()

if("chairs_collection" in db.list_collection_names()):
    chairs.drop()

chairs.insert_many(chair_list)
windows.insert_many(window_list)

# LoraWAN
client_id = "LowPowerIotClientId"
broker = "eu.thethings.network"
appName = "corona_proof_classroom"
appID = "ttn-account-v2._MKGX66ysWDAsAyt9x78jT0wLfV_SyUjXjjptqYqOds"
positionDataType = 8
positionData = 10


def on_connect(client, userdata, flags, rc):
    print("Connected flags ",str(flags),"result code ",str(rc))

def on_message(client, userdata, msg):
    msg = json.loads(msg.payload)
    payloadRaw = msg["payload_raw"]
    payloadPlain = base64.b64decode(payloadRaw).hex()
    parseData(payloadPlain)

def on_subscribe(client,userdata, mid, granted_qos):
	print ("Subscribed: "+ str(mid)+ " with QoS: " + str(granted_qos))

def parseData(sensorData):
    dataType = sensorData[positionDataType:positionData] # Retrieve type of data from payload
    dataPieces = [sensorData[i:i+2] for i in range(positionData, len(sensorData), 2)] # Get all data points
    if dataType == format(ord('T'), 'x'): # Format hex value of T with 0x prefix
        # print('Thingy ID {} neighbour ID: {} RSSI: {}'.format(int(dataPieces[0], 16), int(dataPieces[1], 16), int(dataPieces[2], 16)))
        newChair = Chair(int(dataPieces[0], 16), int(dataPieces[1], 16), int(dataPieces[2], 16))
        if chairs.count_documents ({'_id': newChair._id}, limit=1):
            # Update existing document
            chairs.update_one({"_id": newChair._id},
                {"$set": { "neighbour_id": newChair.neighbour_id, "rssi": newChair.rssi}}            
            )
        else:
            # Insert new document
            chairs.insert_one(eval(json.dumps(newChair.__dict__)))

    elif dataType == format(ord('W'), 'x'):
        # print('Window ID {} status: {} '.format(int(dataPieces[0], 16), dataPieces[1], 16))
        newWindow = Window(int(dataPieces[0], 16), int(dataPieces[1], 16))
        if windows.count_documents ({'_id': newWindow._id}, limit=1):
            # Update existing document
            windows.update_one({"_id": newWindow._id},
                {"$set": { "status": newWindow.status}}            
            )
        else:
            # Insert new document
            windows.insert_one(eval(json.dumps(newWindow.__dict__)))


mqtt_client = mqtt.Client(client_id = client_id , clean_session = True, userdata = None, protocol = mqtt.MQTTv31)
mqtt_client.username_pw_set(appName, appID)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect
mqtt_client.on_subscribe = on_subscribe
mqtt_client.connect(broker, 1883)
mqtt_client.subscribe("+/devices/+/up")

while True:
    try:
        mqtt_client.loop_start()
    except KeyboardInterrupt:
        break
    except Exception:
        break
mqtt_client.loop_stop()