#!/usr/bin/env python

from cv_bridge import CvBridge
import cv2
from dateutil import tz
from mongodb_store.message_store import MessageStoreProxy
import pickle
import pymongo
from smach_msgs.msg import SmachContainerStatus

# FIXME no global
db_name = 'jsk_robot_lifelog'
col_name = "go_to_kitchen"

# Pymongo global var
client = pymongo.MongoClient("mongodb://musca.jsk.imi.i.u-tokyo.ac.jp")
database = client["jsk_robot_lifelog"]
collection = database["go_to_kitchen"]

JST = tz.gettz('Asia/Tokyo')
bridge = CvBridge()


msg_store = MessageStoreProxy(database=db_name, collection=col_name)
last_msg = msg_store.query(
    SmachContainerStatus._type,
    {"active_states": "INIT"},
    single=True,
    sort_query=[("_meta.inserted_at", pymongo.DESCENDING)]
)

msgs = msg_store.query(
    SmachContainerStatus._type,
    {"header.stamp.secs": {"$gt": last_msg[0].header.stamp.secs}},
    sort_query=[("_meta.inserted_at", pymongo.ASCENDING)]
)

pickle.loads(msgs[5][0].local_data)
