#!/usr/bin/python
import os
import logging
import logging.config
import logging.handlers
import math
import sys
       
        
def configure_logging(logname, logfilename,  fsize=100):
    
    INFO1_LEVELV_NUM =  logging.INFO+1
    #Add new log level
    logging.addLevelName(INFO1_LEVELV_NUM, "INFO1")
    logging.basicConfig(
#           level=logging.ERROR, ## with this level peewee will print all the SQL commands
#            level=logging.DEBUG, ## with this level peewee will print all the SQL commands
#            level=logging.INFO+1,## 
            level=logging.INFO,##
            format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s');
#                filename = '/tmp/csmas.log',
#                filemode='a')
    def info1(self, message, *args, **kws):
    #    # Yes, logger takes its '*args' as 'args'.
        lvl=logging.getLogger().getEffectiveLevel()
        if lvl <= INFO1_LEVELV_NUM:   
            self._log(INFO1_LEVELV_NUM, message, args, **kws) 
    logging.Logger.info1 = info1
  
    #filelog = logging.FileHandler(logfilename, 'a');
    filelog = logging.handlers.RotatingFileHandler(logfilename, mode='a', maxBytes=fsize*math.pow(10,6), backupCount=2);
    formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
    filelog.setFormatter(formatter)
    filelog.setLevel(logging.INFO) ## with this level peewee will print all the SQL commands


    # define a Handler which writes INFO messages or higher to the sys.stderr
    consoleerr = logging.StreamHandler(sys.stderr)
    consoleerr.setLevel(logging.INFO)
# set a format which is simpler for console use
#       formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
    # tell the handler to use this format
    consoleerr.setFormatter(formatter)


    # define a Handler which writes INFO messages or higher to the sys.stdout
    consoleout = logging.StreamHandler(sys.stdout)
    consoleout.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')
    # tell the handler to use this format
    consoleout.setFormatter(formatter)
 
    # add the handler to the root logger
#    logging.getLogger('').addHandler(consoleerr)
#    logging.getLogger('').addHandler(consoleout)
    logging.getLogger(logname).addHandler(filelog)
#    logging.config.fileConfig('/tmp/csmas.log');
    print ('logname={}'.format(logname));
    log = logging.getLogger(logname)
    

    return log

#log = configure_logging()

