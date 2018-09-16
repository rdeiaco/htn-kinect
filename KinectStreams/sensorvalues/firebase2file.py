import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import db
import time

# Use a service account
cred = credentials.Certificate("H:\HTN2018\htn-kinect\KinectStreams\sensorvalues\htn2018-acba7-firebase-adminsdk-ictsm-1548e14db6.json")
firebase_admin.initialize_app(cred)

db = firestore.client()

base1_ref = db.collection(u'targets').document(u'base1')
base2_ref = db.collection(u'targets').document(u'base2')
base3_ref = db.collection(u'targets').document(u'base3')
print(base1_ref.get()._data['signal'])
print(base2_ref.get()._data['signal'])
print(base3_ref.get()._data['signal'])

while(True):
	base1_signal = base1_ref.get()._data['signal']
	base2_signal = base2_ref.get()._data['signal']
	base3_signal = base3_ref.get()._data['signal']

	file = open("sensor_values.csv","w")
	file.write(str(base1_signal) + "," + str(base2_signal) + "," + str(base3_signal))
	file.close()
	time.sleep(0.1)
 

