#!/usr/bin/env python

import rospy
import rosparam

def printDict( dictionary ):
    length = len(dictionary);
    keys = dictionary.keys();
    keys_ln = len(keys);
    print ("A dict with ln: %d"%(length))

    for i in range(keys_ln):
        name = keys[i];
        c    = dictionary[name]
        print name;
        print c;

def printList( a_list ):
    length = len(a_list);
    print ("A list with ln: %d"%(length))
    for i in range(length):
        print a_list[i];

if __name__ == '__main__':
    try:
        param_list =  rosparam.load_file("mtb_path.txt");
        param_dictionary = param_list[0][0];
        param_namespace = param_list[0][1];
        param_keys = param_dictionary.keys();
        num_keys = len(param_keys);

        for i in range(num_keys):
            keyname = param_keys[i];
            value   = param_dictionary[keyname];
            value_class = value.__class__;
            print ("Key '%s' is a %s : "%(keyname,str(value_class)));

            if value_class == dict:
                printDict  (value);
            elif value_class == list:
                printList(value);
            else:
                print value;
                pass;
            print '\n'


    #    print str(string);
    except rosparam.RosParamException as ex:
        print ex.message;
