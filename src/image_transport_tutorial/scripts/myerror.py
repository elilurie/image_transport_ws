#!/usr/bin/env python
# myerror.py
###################################################### 
import sys
import os
import logging
import json
import datetime
from datetime import datetime, timedelta
import time
import re
import uuid
import argh
from argh import arg
import math
import requests
lib_path = os.path.abspath(os.path.join('../common'))
sys.path.append(lib_path)
#import db_utils
#import sendmsg_utils
#import myutils
#import apicache
import copy
# My SQL tables
#
import binascii

#####################
# lineno
#####################
def lineno():
    exc_type, exc_obj, exc_tb = sys.exc_info()
    return exc_tb.tb_lineno

#################################
# class ValueErrorUserMessage
#################################
class ValueErrorUserMessage(Exception):
    """Raised when the input value is too small"""
    def __init__(self, msg):
        super(ValueErrorUserMessage, self).__init__(msg)
        self.msg=msg   
################################
# class CDBError
################################
class CDBError():
    def __init__(self, log):
        self.log=log
    def write_error(self,errcode=None, supid=None, assetid=None, sensorid=None, errmsg=None):
        fn='CDBError::write_error()'
        try:
            crc32val=(binascii.crc32(errmsg) % (1<<32)) & 0xFFFFFFFF
            crc32str = '{:X}'.format(crc32val);

            self.log.info('{}: supid:{} errcode:{} assetid:{} errmsg:{} crc32str:{}'\
                    .format(fn, supid, errcode, assetid, errmsg, crc32str))
            try:
                rec=Errors.select()\
                    .where(Errors.Supplier_ID==supid, 
                        Errors.Asset_ID==assetid,
                        Errors.Error_Code==errcode)\
                    .order_by(-Errors.id).get()
                self.log.info('{}: Latest error was found in Errors_Table with id:{} crc:{}. Compare the CRC to:{}'\
                            .format(fn, rec.id, rec.Err_Crc, crc32str))
                if rec.Err_Crc==crc32str:
                    self.log.info('{}: rec.Err_Crc=crc32str:{}=>Do not add this error to the table'.format(fn, crc32str))
                    return False;
                else:
                    self.log.info('{}: rec.Err_Crc:{}!=crc32str:{}=>Add this error to the table'\
                        .format(fn, rec.Err_Crc, crc32str))
                    
            except Exception as err:
                self.log.warning('{}: Failed!!! error was not found in Errors_Table=>add this error'.format(fn))                    

            rec=Errors()
            rec.Supplier_ID=supid
            rec.Asset_ID=assetid
            rec.Error_Code=errcode
            rec.Err_Msg=errmsg
            rec.Sensor_ID=sensorid
            rec.Err_Crc=crc32str
            rec.save();
            return True
        except Exception as err:
            msg='{}: Failed!!! err:{}'.format(fn, err);
            self.log.error(msg)            
            raise ValueError(msg)
##########################################
##########################################
# PROCEDURES
##########################################
##########################################

###########################################
#  writeerr
#   Write err
#  usage:
#       ./myerror.py writeerr -s 21 -e ['ERR_GENERAL','ERR_PARSE_DELIVERY_DATES'] -a 5681071 -m 'Some message...' 

###########################################
#@arg('supid',help='Supplier ID' )
def writeerr(supid=None, errcode=None, assetid=None, msg=None):
    '''Write error into DB'''
    fn='writeerr()'
    try:
        log = configure_logging('myerror', "/tmp/myerror.log")
        log.info('{}: supid:{} errcode:{} assetid:{} msg:{}'.format(fn,supid, errcode, assetid, msg))
        obj=CDBError(log);
        obj.write_error(errcode=errcode, supid=supid, assetid=assetid, errmsg=msg);
    except Exception as err:
        msg='{}: Failed!!! err:{}'.format(fn, err)
        log.error(msg)

#       
####################################
# MAIN
####################################
arg_parser = argh.ArghParser()
arg_parser.add_commands([writeerr])
if __name__ == '__main__':
    arg_parser.dispatch()
 

