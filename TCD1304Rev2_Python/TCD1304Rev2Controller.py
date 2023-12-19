#!/usr/bin/python

"""
TCD1304Rev2Controller.py

Mitchell C. Nelson (c) 2023

November 26, 2023

Derived from LLCDController.py Copyright 2022 by M C Nelson
Derived from TDAQSerial.py, TCD1304Serial.py, Copyrigh 2021 by M C Nelson

"""

__author__    = "Mitchell C. Nelson, PhD"
__copyright__ = "Copyright 2023, Mitchell C, Nelson"
__version__   = "0.3"
__email__     = "drmcnelson@gmail.com"
__status__    = "alpha testing"

__all__ = [ 'LCCDFRAME', 'LCCDDATA', 'LCCDDATASET' ]

versionstring = 'LCCDController.py - version %s %s M C Nelson, PhD, (c) 2023'%(__version__,__status__)

import sys
import time
import select

import os
import operator

import signal
import atexit

#from timeit import default_timer as timer

import platform

import serial

from datetime import datetime
from time import sleep, time, process_time, thread_time

# from threading import Lock, Semaphore, Thread
from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value, Lock

import struct

import inspect
from itertools import count

import regex
from pyparsing import nestedExpr

import numpy as np
from scipy.signal import savgol_filter as savgol

import matplotlib.pyplot as plt

try:
    from Accumulators import Accumulators
    has_accumulators = True
except ModuleNotFoundError:
    has_accumulators = False

try:
    from TextWindow import TextWindow
    has_TextWindow = True
except Exception as e:
    has_TextWindow = False
    
try:
    from GUIWindow import GUIWindow
    has_GUIWindow = True
except Exception as e:
    has_GUIWindow = False
    print(e)

try:
    from GraphicsWindow import GraphicsWindow
    has_GraphicsWindow = True
except:
    has_GraphicsWindow = False
    
# ----------------------------------------------------------

def lineno():
    return inspect.currentframe().f_back.f_lineno

def errorprint(*s): 
    print(*s, file = sys.stderr) 

def input_ready():
    return (sys.stdin in select.select([sys.stdin], [], [], 0)[0])

def key_in_list( ls, key, mapf=str ):
    try:
        idx = ls.index(key)
        return mapf( ls[idx+1] )
    except:
        return None

def generate_x_vector( npoints, coefficients = None ):

    x = np.linspace( 0, npoints, npoints )
    
    if coefficients is None:
        return x

    else:
        return np.polynomial.polynomial.polyval( x, coefficients )

# --------------------------------------------------------------------------------------------
def split_nested( s ):
    result = regex.search(r'''
    (?<rec> #capturing group rec
    \( #open parenthesis
    (?: #non-capturing group
    [^()]++ #anyting but parenthesis one or more times without backtracking
    | #or
    (?&rec) #recursive substitute of group rec
    )*
    \) #close parenthesis
    )
    ''',s)
    return result

#matches = [match.group() for match in regex.finditer(r"(?:(\((?>[^()]+|(?1))*\))[\S)+",s)]
#return matches

def split_bracketed(string, delimiter=' ', strip_brackets=False):
    """ Split a string by the delimiter unless it is inside brackets.
    e.g.
        list(bracketed_split('abc,(def,ghi),jkl', delimiter=',')) == ['abc', '(def,ghi)', 'jkl']

    stackoverflow question 21662474, answer by Peter, 10/5/21
    """

    openers = '[{(<'
    closers = ']})>'
    opener_to_closer = dict(zip(openers, closers))
    opening_bracket = dict()
    current_string = ''
    depth = 0
    for c in string:
        if c in openers:
            depth += 1
            opening_bracket[depth] = c
            if strip_brackets and depth == 1:
                continue
        elif c in closers:
            assert depth > 0, f"You exited more brackets than we have entered in string {string}"
            assert c == opener_to_closer[opening_bracket[depth]], \
                f"Closing bracket {c} did not match opening bracket {opening_bracket[depth]} in string {string}"
            depth -= 1
            if strip_brackets and depth == 0:
                continue
        if depth == 0 and c == delimiter:
            yield current_string
            current_string = ''
        else:
            current_string += c
    assert depth == 0, f'You did not close all brackets in string {string}'
    yield current_string    

# ======================================================================================================================
# ======================================================================================================================
# Classes for reading data from disk file

# This is the individual frame,  the framesets class has a list of frames

class LCCDFRAME:

    def __init__( self, content, parent, offsetdigits=6, dtype=None ):

        self.parent = parent

        self.accumulate = 0
        
        while len(content):

            line = content[0].lower().strip()
            if not len(line):
                content = content[1:]
                continue

            # We found the data
            if line.lower().startswith( '# data ascii' ):
                self.datalen = int(line.split()[3] )
                data = []
                n = 1
                for l in content[1:]:
                    n += 1
                    if l.lower().startswith( '# end data' ):
                        break
                    data.append( float( l ) )
                self.data = np.array(data)
                content = content[n:]
                #print( 'completed data', len(data) )

            # Frame header and supplemental (after the data, until the blank lines)
            elif line[0] == '#':

                line = line[1:].strip()

                if '=' in line:
                    exec( line, self.__dict__ )

                # We simply load the dictionary from the keyword value pairs in the frame header
                elif line.lower().startswith( 'timestamp' ):
                    key,value = line.split(maxsplit=1)
                elif key == 'adcdata':
                    self.__dict__[key] = [ a for a in map( float, value.split() ) ]
                    
                content = content[1:]
                
            else:
                print( 'Warning: empty line in frame' )

                content = content[1:]

        for key in ['INTERVAL','CLOCK','MODE','SETS','ACCUMULATE','OUTERCOUNTER',
                    'interval','clock','mode','sets','accumulate','outercounter']:
            if key in self.__dict__ and key not in parent.__dict__:
                parent.__dict__[key] = self.__dict__[key]

        if 'ELAPSED' in self.__dict__:
            self.__dict__['offset'] = self.ELAPSED/1.E6
        elif 'elapsed' in self.__dict__:
            self.__dict__['offset'] = self.elapsed/1.E6
        elif 'TRIGGERELAPSED' in self.__dict__:
            self.__dict__['ELAPSED'] = self.TRIGGERELAPSED
            self.__dict__['offset'] = self.TRIGGERELAPSED/1.E6
        elif 'tiggerelapsed' in self.__dict__:
            self.__dict__['triggerelapsed'] = self.triggerelapsed
            self.__dict__['offset'] = self.triggerelapsed/1.E6
            
        if 'offset' in self.__dict__ and offsetdigits:
            self.offset = round( self.offset,offsetdigits)

    def __len__(self):
        return len(self.data)
    
    def get(self,s):
        if s in self.__dict__:
            return self.__dict__[s]
        else:
            exec( 'retv='+s, self.__dict__ )
            return retv
    
    # ------------------------------------------------------------------
    def dump(self):
        for key,val in self.__dict__.items():
            if key == 'parent' and val is not None:
                print( key, self.parent.filename )
            elif key != '__builtins__':
                print( key, val )
    # ------------------------------------------------------------------
 

# ---------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------

class LCCDDATA:

    def __init__( self, filename, dtype=None, relatedobject=None, offsetdigits=6, verbose=False ):

        self.filename = filename
        self.relatedobject = relatedobject

        with open(filename,'r') as f:
            content = f.read().splitlines()

        if not content[0].startswith( "# LCCD" ):
            print( content[0] )
            raise ValueError( "file is not LCCD" );
        
        # Read the header
        self.version = content[0]
        self.filedate = content[1]
        self.chiptemperature = None
        self.coefficients = None

        for n,line in enumerate(content[2:],start=2):

            if line[0] != '#':
                raise ValueError( 'lines in file header need to start with #' )

            line = line[1:].strip()
            
            if line.lower().startswith("header end" ):
                content = content[n+1:]
                break

            # ===============================
            # these are the version 0 files
            if line.startswith( "shutter" ):
                content = content[n:]
                break
            # ==============================
            
            if '=' in line:
                exec( line, self.__dict__ )

            # ==============================
            # version 0 parameter specfications
            else:
                if ':' in line:
                    key,value = line.split(':',maxsplit=1)
                else:
                    key,value = line.split(maxsplit=1)

                if key == 'coefficients':
                    self.coefficients = [ a for a in map( float, value.split() ) ]
                else:
                    try:
                        self.__dict__[key] = int( value )
                    except ValueError:
                        try:
                            self.__dict__[key] = float( value )
                        except ValueError:
                            # this is the last resort, it if fails, need to fix the file
                            self.__dict__[key] = str( value )

        # ------------------------------------------
        # Post header fixups
        if 'datalength' in self.__dict__:
            self.xdata = generate_x_vector( self.datalength, self.coefficients )
            self.wavelengths = self.xdata
            self.pixels = np.linspace( 0, self.datalength, self.datalength )

        # ==========================================
        # Load the frames
        self.frames = []

        while( len(content) ):

            # Skip blank lines
            for n, line in enumerate(content):
                line = line.strip()
                if len(line):
                    break
            content = content[n:]
            if not len(content):
                break

            # Find next blank lines
            for m, line in enumerate(content):
                line = line.strip()
                if len(line) == 0:
                    break
            if not len(content[:m]):
                break
            
            frame = LCCDFRAME(content[:m],self,offsetdigits)
            if len(frame):
                self.frames.append(frame)
            else:
                raise ValueError('empty frame')

            content = content[m:]

        # ------------------------------------------
        # Adjust offsets for pulse leadin
        if 'pulse_leadin' in self.__dict__ :
            offset = self.frames[self.pulse_leadin].offset;
            for f in self.frames:
                f.offset -= offset
                if offsetdigits:
                    f.offset = round(f.offset,offsetdigits)

    def __len__(self):
        return len(self.frames)

    def get(self,s):
        if s in self.__dict__:
            return self.__dict__[s]
        else:
            exec( 'retv='+s, self.__dict__ )
            return retv
    
    def getlist( self, attr_name ):

        rets = []
        for f in self.dataset:
            rets.append( f.get(attr_name) )
            
        try:
            rtemp = np.array(rets)
            rets = rtemp
        except:
            pass

        return rets
    
    # ------------------------------------------------------------------
    def dump(self):
        print( "***************************" )
        for key,val in self.__dict__.items():
            if key == 'relatedobject' and val is not None:
                if 'filename' in self.relatedobject.__dict__:
                    print( key, self.relatedobject.filename )
            
            elif key not in ['__builtins__','frames']:
                print( key, val )

        for n,frame in enumerate(self.frames):
            print( "---------------------------" )
            print( "*frame", n )
            frame.dump()

        if self.relatedobject is not None:
            try:            
                self.relatedobject.dump()
            except Exception as e:
                print(e)

# ---------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------
class LCCDDATASET:

    def __init__( self, filespecs=None, dataset=None, sort_attr=None, verbose=False ):

        self.dataset = []

        if filespecs is not None:
            
            if type(filespecs) is not list:
                filespecs = [ filespecs ]
        
            for filespec in filespecs:

                for filespec_ in glob.glob(filespec):

                    if filespec_.endswith('lccd'):
                        lccd = LCCDDATA( filespec_, verbose=verbose )
                        self.dataset.append(lccd)

        if dataset is not None:
            for d in dataset:
                if isinstance(d,LCCDDATA):
                    self.dataset.append(d)
                else:
                    raise ValueError( 'not LCCDDATA instance' )

        if sort_attr is not None:
            self.dataset.sort( key=operator.attrgetter( sort_attr ) )

    def get(self,s):
        if s in self.__dict__:
            return self.__dict__[s]
        else:
            exec( 'retv='+s, self.__dict__ )
            return retv
    
    def getlist( self, attr_name ):

        rets = []
        for d in self.dataset:
            rets.append( d.get(attr_name) )
            
        try:
            rtemp = np.array(rets)
            rets = rtemp
        except:
            pass

        return rets
            
    def sort( self, attr_name ):
        print( 'sort', attr_name )
        self.dataset.sort( key=operator.attrgetter( attr_name ) )


    def slices( self, n, currentnorm=False ):

        slices_ = []
        for d in self.dataset:
            if currentnorm:
                slices_.append( d.frames[n].data/d.relatedobject.avgcurrent )
            else:
                slices_.append( d.frames[n].data )

        return np.array(slices_)

    def __len__(self):
        return len(self.dataset)
    
# ======================================================================================================================
# ======================================================================================================================
# This is the class for the instrument

class LCCDCONTROLLER:

    _ids = count(0)
    
    def __init__( self, portspec, readtimeout=1., writetimeout=1., monitor=True, graphics=True,
                  graph_by_pixels=False, xrange=None, yrange=None, graph_ylabel='spectrum',
                  coefficients=None, gui=False,
                  debug=False ):

        
        if gui and not has_GUIWindow:
            raise ValueError( "GUI requested, but GUIWindow.py not loaded" )
            
        if graphics and not has_GraphicsWindow:
            raise ValueError( "Graphics requested, but GraphicsWindow.py not loaded" )
            
        if monitor and not has_TextWindow:
            raise ValueError( "Monitor requested, but TextWindow.py not loaded." )
            
        # ------------------------------------------------------------------
        self.ser = serial.Serial( portspec, timeout=readtimeout, write_timeout=writetimeout )

        self.instance = next( self._ids )
        
        self.name= portspec
        
        self.textqueue = Queue()
        self.dataqueue = Queue()
        
        self.monitorthread = None
        self.monitorWindow = None
        self.monitorqueue = Queue()
        
        self.GrapichsWindow = None
        self.xdata = None

        self.flag = Value( 'i', 1 )
        self.busyflag = Value( 'i', 0 )
        self.accumulatorflag = Value( 'i', 0 )

        # Local accumulators for the command line
        self.accumulators = Accumulators( self.dataqueue, ycol0=0, parentinstance = self )
                
        # Control data processing in reader
        self.baselineflag = Value( 'i', 1 )

        # Report errors back
        self.errorflag = Value( 'i', 0 )

        self.bits = 12
        self.vfs = 3.3
        self.vperbit = self.vfs/(2**self.bits - 1)
        
        # --------------------------------
        self.filesuffix = ".lccd"

        self.identifier = None
        self.coefficients = []

        self.graph_by_pixels = graph_by_pixels

        # ---------------------------------

        self.debug = debug
        
        # ---------------------------------
        # First, stop
        buffer = self.rawcommand( 'stop' );
        if buffer is not None:
            print( buffer )
        
        # ---------------------------------
        # Query for Identifier        
        buffer = self.rawcommand( 'identifier', 'Identifier' );
        if buffer is not None:
            print( buffer )
            self.identifier = buffer.split(maxsplit=1)[1]
        
        # ---------------------------------
        # Query for the configuration
        buffer = self.rawcommand( 'configuration', 'PIXELS' );
        if buffer is None:
            raise ValueError( "configuration, not found in response" )
        print( buffer )
        
        parts = buffer.split()
        self.datalength = key_in_list( parts, "PIXELS", int )
        self.darklength = key_in_list( parts, "DARK", int )
        self.invert = key_in_list( parts, "INVERT", float )
        self.sensor = key_in_list( parts, "SENSOR", str )
        
        if  "VPERBIT" in parts:
            self.vperbit = key_in_list( parts, "VPERBIT", float )
            print( "Vperbit", self.vperbit )
            
        if "BITS" in parts and "VFS" in parts :
            self.bits = key_in_list( parts, "BITS", int )
            self.vfs = key_in_list( parts, "VFS", float )
            self.vperbit = self.vfs/(2**self.bits - 1)
            print( "BITS", self.bits, "VFS", self.vfs, "Vperbit", self.vperbit )

        print( "pixels", self.datalength,
               "dark", self.darklength,
               "invert", self.invert,
               "vperbit", self.vperbit,
               "sensor", self.sensor )

        # ----------------------------------------
        # Query for coefficients
        
        self.xdata = np.linspace( 0, self.datalength, self.datalength )
        self.xlabel = 'Pixels'

        if coefficients is not None:
            print( 'using specified coefficients', self.coefficients )
            self.coefficients = coefficients
            self.xlabel = 'user coords'
            
        else:
            buffer = self.rawcommand( 'coefficients', 'coefficients' );
            if buffer is not None:
                if self.debug:
                    print( 'coefficients buffer', buffer )
                # Also sets self.xdata
                if self.parsecoefficients( buffer ):
                    self.xlabel = 'Wavelength'
                print( 'coefficients', self.coefficients )
                print( self.xdata )
            

        # ---------------------------------
        if xrange is None:
            xrange = (self.xdata[0],self.xdata[-1])
        if yrange is None:
            yrange = (-self.vfs/20,self.vfs)
            
        if gui:
            # We will want to query these from the device
            self.GraphicsWindow = GUIWindow( "LCCDController Data " + portspec,
                                             xdata = self.xdata,
                                             xlabel = self.xlabel,
                                             ycols = [ np.zeros(self.datalength) ],
                                             yrange = (-self.vfs/20,self.vfs),
                                             ylabels = [graph_ylabel],
                                             flag = self.flag,
                                             parentinstance = self,
                                             filespec = os.path.join( 'datafile', self.filesuffix ),
                                             debug = self.debug )
            self.GraphicsWindow.start( )

        elif graphics:
            # We will want to query these from the device
            self.GraphicsWindow = GraphicsWindow( "LCCDController Data " + portspec,
                                                  xdata = self.xdata,
                                                  xlabel = 'wavelength',
                                                  ycols = [ np.zeros(self.datalength) ],
                                                  yrange = (-self.vfs/20,self.vfs),
                                                  ylabels = [graph_ylabel],
                                                  flag = self.flag,
                                                  debug = self.debug )
            self.GraphicsWindow.start( )
        
        if monitor:
            self.monitorthread = Process( target = self.textmonitor, args=(portspec, self.flag ) )
            self.monitorthread.start()
            
        self.readerthread = Process( target = self.reader,
                                     args=(portspec, self.flag, self.busyflag, self.accumulatorflag,
                                           self.baselineflag, 
                                           self.errorflag, self.debug ) )
        self.readerthread.start()

        atexit.register(self.exit, None )

    # ------------------------------------------
    def parsecoefficients( self, buffer ):

        print( buffer )
        if 'nan' in buffer:
            self.coefficients = [ 0., 1., 0., 0. ]
            self.xdata = generate_x_vector( self.datalength, None )
            return False
        
        try:
            self.coefficients = [ a for a in map( float, buffer.split()[1:] ) ]            
            self.xdata = generate_x_vector( self.datalength, self.coefficients )
        except Exception as e:
            print( e )
            return False

        return True
    
    # ===========================================
    def rawcommand( self, command, key=None ):

        print( "sending", command )
        self.write( command + '\n' );

        response = []
        while True:
            try:
                buffer = self.ser.read_until( )
                buffer = buffer.decode()[:-1]
                print( "rcvd: ", buffer )
            except Exception as e:
                print( "rawread", e )
                break            
            if buffer.startswith( "DONE" ):
                break
            response.append( buffer )

        if key is not None:
            print( 'rawcommand scanning response for ', key )
            # return the selected line or None
            candidate = None
            for s in response:
                print( 'rawcommand line:', s )
                if s.startswith( key ):
                    candidate = s
            if candidate is not None:
                return candidate
            print( key, ' not found in response', response )
            return None
            
        return response

    # ===========================================
    def exit( self, ignored=None ):
        print( self.name + ' exit()' )
        self.write( 'stop\n' );
        self.close()

    def busy(self):
        return self.busyflag.value

    # Wait for completion of data frames
    def wait(self, timeout=None, interruptible=False ):

        if timeout is None:
            while self.busyflag.value:
                if interruptible and input_ready():
                    return False
                sleep( 0.2 )
            return True
        
        try:
            timeout = float(timeout)
        except:
            print( "not valid timeout value", timeout )
            return False
            
        while self.busyflag.value:
            if interruptible and input_ready():
                return False
            elif timeout > 0.:
                sleep( 0.2 )
                timeout -= 0.2
            else:
                return False

        return True
        
    # ===========================================
    def textmonitor( self, name, flag ):

        print( "text monitor started ", name )

        self.monitorWindow = TextWindow("LCCDSpectrometer Log " + name)
        print( self.monitorWindow )

        def on_closing():
            print( 'closed' )
            flag.value = 0
        #self.monitorWindow.parent.protocol("WM_DELETE_WINDOW", on_closing)
        self.monitorWindow.parent.protocol("WM_DELETE_WINDOW", self.monitorWindow.parent.iconify)            
        
        def sighandler( a, b ):
            flag.value = 0
        signal.signal(signal.SIGINT, sighandler)

        while flag.value:
            try:
                line = self.monitorqueue.get(block=False)
                self.monitorWindow.addline_( line )
            except Empty:
                pass
            self.monitorWindow.update()
            sleep(0.1)
        self.monitorWindow.close()

    # =======================================
    def enqueueGraphics(self, record ):

        ycols, \
            frame_shutter, frame_interval, frame_clock, \
            frame_counter, frame_outercounter, \
            frame_frames, frame_sets, frame_every, \
            frame_mode, frame_elapsed, \
            accumulate, timestamp = record

        text = frame_mode + ' ' + str(frame_shutter) + ' ' + str(frame_interval) + '/' + str(frame_clock)
        text += '\ncounters: ' + str(frame_counter) + '/' + str(frame_frames)
        text += ' ' + str(frame_outercounter) + str(frame_sets)
        text += '\n' + timestamp.strftime('%Y-%m-%d.%H%M%S.%f')
        if accumulate:
            text += '\n' + str(accumulate)

        #print( "enqueue graphics" )
        self.GraphicsWindow.queue.put( [ ycols, text ] )
        #self.GraphicsWindow.queue.put( record  )
        
        return True
    
        
    def reader( self, name, flag, busyflag, accumulatorflag, baselineflag, errorflag, debug=False ):

        # ----------------------------------------------
        def sighandler( a, b ):
            print( "reader sighandler" )
            flag.value = 0
            
        signal.signal(signal.SIGINT, sighandler)

        # ----------------------------------------------
        naccumulators = 0
        accumulators = []
        accumulation_counters = []
        accumulators_elapsed = []

        # Need fast access to these, do the "." now
        datalength = self.datalength
        dataqueue_get = self.dataqueue.get
        dataqueue_put = self.dataqueue.put
        dataqueue_empty = self.dataqueue.empty

        graphics_put = self.GraphicsWindow.queue.put
        
        def initializeaccumulators():
            nonlocal accumulators
            nonlocal accumulation_counters
            nonlocal accumulators_elapsed
            nonlocal naccumulators
            nonlocal datalength
            
            print( "setting up accumulators", frame_frames )
            naccumulators = frame_frames
            accumulators = [ np.zeros(datalength) ] * naccumulators
            accumulation_counters = [ 0 ] * naccumulators
            accumulators_elapsed = [0 ] * naccumulators
            
        def enqueueaccumulators():
            nonlocal accumulators
            nonlocal accumulation_counters
            nonlocal accumulators_elapsed
            nonlocal naccumulators            
            
            for nindex,(data,counter,elapsed) in enumerate(
                    zip(accumulators,accumulation_counters,accumulators_elapsed) ):

                if counter > 1:
                    data /= counter
                    
                record = [ [data],
                           frame_shutter, frame_interval, frame_clock,
                           nindex+1, frame_outercounter,
                           frame_frames, frame_sets, frame_every,
                           frame_mode, frame_elapsed,
                           counter, timestamp ]
                
                dataqueue_put( record )

        def recordprocessing( ):
            nonlocal accumulators
            nonlocal accumulation_counters
            nonlocal accumulators_elapsed
            nonlocal naccumulators            
            nonlocal data
            
            if accumulate:

                nindex = (frame_counter - 1)
                    
                print( 'accumulating index', nindex, naccumulators )
                counter = accumulation_counters[nindex ]

                accumulators[nindex] += data
                accumulation_counters[nindex] += 1

                oldelapsed = accumulators_elapsed[nindex]
                accumulators_elapsed[nindex] = frame_elapsed

                if accumulation_counters[nindex] > 1:
                    data = accumulators[nindex] / accumulation_counters[nindex]
                    if oldelapsed != frame_elapsed:
                        print( "Warning: elapsed times differ", oldelapsed, frame_elapsed )
                
            else:
                record = [ [data],
                           frame_shutter, frame_interval, frame_clock,
                           frame_counter, frame_outercounter,
                           frame_frames, frame_sets, frame_every,
                           frame_mode, frame_elapsed,
                           0, timestamp ]

                dataqueue_put( record )

            #  ----------------------------------------
            if self.GraphicsWindow:
                text = frame_mode
                text += ' ' + str(frame_shutter)
                text += ' ' + str(frame_interval)
                text += '/' + str(frame_clock)
                text += ' ' + str(frame_elapsed)
                text += '\ncounters: '
                text += str(frame_counter) + '/' + str(frame_frames)
                text += ' ' + str(frame_outercounter) + '/' + str(frame_sets)
                text += '\n' + timestamp.strftime('%Y-%m-%d.%H%M%S.%f')
                if accumulate:
                    text += '\n' + str(accumulation_counters[nindex])

                #print( "enqueue graphics" )
                graphics_put( [ [data], text ] )
        
        def initialize_recordprocessing():

            if debug:
                print( 'lccd setting busyflag' )
            busyflag.value = 1
            try:
                while not dataqueue_empty():
                    dataqueue_get()
            except Exception as e:
                print( 'clearing dataqueue', e )

            if accumulatorflag.value:
                initializeaccumulators()
                accumulate = True

        # -------------------------------------------------
        frame_mode = ""
        frame_elapsed = 0
        frame_interval = 0
        frame_shutter = 0
        frame_clock = 0                
        frame_counter = 0
        frame_outercounter = 0
        frame_frames = 0
        frame_every = 0
        frame_sets = 0
        frameset_complete = False

        accumulate = False
        accumulate_counter = 0

        frameset_complete = False
        
        def initialize_parameters():
            nonlocal frame_elapsed
            nonlocal frame_interval
            nonlocal frame_shutter
            nonlocal frame_clock                
            nonlocal frame_counter
            nonlocal frame_outercounter
            nonlocal frame_frames
            nonlocal frame_every
            nonlocal frame_sets
            nonlocal frameset_complete
            frame_elapsed = 0
            frame_interval = 0
            frame_shutter = 0
            frame_clock = 0                
            frame_counter = 0
            frame_outercounter = 0
            frame_frames = 0
            frame_every = 0
            frame_sets = 0
            frameset_complete = False

                
        # ----------------------------------------------
        print( "lccd reader start", name, 'debug', debug )
                
        read_until = self.ser.read_until
        
        while flag.value:

            buffer = read_until( )
            
            if buffer is not None and len(buffer) >1 and flag.value:

                try:
                    buffer = buffer.decode()[:-1]
                except:
                    print( 'failed decode', buffer )
                    continue

                if debug:
                    print( "lccd reader: ", buffer )

                if buffer.startswith( "DONE" ):
                    self.textqueue.put( buffer )
                
                # Receive Ascii Formatted Data
                elif buffer.startswith( "DATA" ):
                    ndata = int(buffer[4:])
                    #print( 'ndata', ndata )

                    # Read the actual text format data buffer(s)
                    data_buffers = []
                    while len(data_buffers) < ndata:
                        data_buffers.append( self.ser.read_until( ) )

                    timestamp = datetime.now()

                    # Read(Expect) the end of data message
                    endbuffer = self.ser.read_until( )
                    endbuffer = endbuffer.decode()[:-1]

                    if debug:
                        print( "lccd endbuffer: ", endbuffer )
                        
                    # If valid, process the data
                    if endbuffer.startswith( "END" ):
                        
                        data = []
                        for b in data_buffers:
                            data.append( int(b.decode()) )
                            
                        data = np.array(data)

                        if self.vperbit:
                            data = data * self.vperbit

                        if baselineflag.value and self.darklength:
                            data -= np.sum( data[:self.darklength] ) / self.darklength

                        #  ----------------------------------------
                        recordprocessing()
                        #  ----------------------------------------

                    else:
                        print('reader ' + name +  ' ', buffer, len(data), ' without END')
                        self.textqueue.put( "ERROR: data not completed" )
                        if self.monitorquue:
                            self.monitorqueue.put( "ERROR: data not completed\n" )  

                # --------------------------------
                # Receive Binary Formatted Data Buffer
                elif buffer.startswith( "BINARY16" ):
                    ndata = int(buffer[8:])
                    #print( 'ndata', ndata )

                    # Read the data
                    data = self.ser.read( ndata*2 )

                    timestamp = datetime.now()

                    # Read(Expect) the end of data message
                    endbuffer = self.ser.read_until( )
                    endbuffer = endbuffer.decode()[:-1]

                    # Update the text display, begin and end mesages
                    if debug:
                        print( "endbuffer: ", endbuffer )
                    
                    if endbuffer.startswith( "END" ):
                        
                        data = struct.unpack( '<%dH'%(len(data)/2), data )

                        data = np.array(data)
                        
                        if self.vperbit:
                            data = data * self.vperbit

                        if baselineflag.value and self.darklength:
                            data -= np.sum( data[:self.darklength] ) / self.darklength
                            
                        #data = self._mapdata( data )

                        #  ----------------------------------------
                        recordprocessing()
                        #  ----------------------------------------
                        
                    else:
                        print('reader ' + name +  ' ', buffer, len(data), ' without END')
                        
                        self.textqueue.put( "ERROR: data not completed" )
                        if self.monitorqueue:
                            self.monitorqueue.put( "ERROR: data not completed\n" )

                # ===================================================
                # Command flags
                elif buffer.startswith( "ELAPSED" ):

                    try:
                        frame_elapsed = int(buffer[7:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "COUNTER" ):

                    #print( buffer )
                    try:
                        frame_counter = int(buffer[7:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "SHUTTER" ):
                    #print( buffer )
                    try:
                        frame_shutter = int(buffer[8:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                # ---------------------------------------
                # This come at the start and end of each frameset
                elif buffer.startswith( "FRAMESET START" ):
                    frameset_complete = False
                    try:
                        frame_outercounter = int(buffer[14:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                elif buffer.startswith( "FRAMESET END" ):
                    frameset_complete = True

                # ---------------------------------------
                # Common specs
                # note, we need the space after keyword for this one
                elif buffer.startswith( "CLOCK " ):
                    #print( buffer )
                    try:
                        frame_clock = int(buffer[5:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                                
                elif buffer.startswith( "INTERVAL" ):
                    #print( buffer )
                    try:
                        frame_interval = int(buffer[8:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                        
                elif buffer.startswith( "FRAMES" ):
                    #print( buffer )
                    try:
                        frame_frames = int(buffer[6:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                
                elif buffer.startswith( "SETS" ):
                    #print( buffer )
                    try:
                        frame_sets = int(buffer[4:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                    initialize_recordprocessing()
                                            
                elif buffer.startswith( "EVERY" ):
                    #print( buffer )
                    try:
                        frame_every = int(buffer[ 5:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )
                
                # ---------------------------------------
                # Triggered
                elif buffer.startswith( "TRIGGERED SINGLES START" ):
                    initialize_parameters()
                    frame_mode = "TRIGGER"

                elif buffer.startswith( "TRIGGERED SETS START" ):
                    initialize_parameters()
                    frame_mode = "TRIGGERSETS"
                    
                    initialize_recordprocessing()
                        
                # ---------------------------------------
                # Clocked
                elif buffer.startswith( "CLOCKED START" ):
                    initialize_parameters()
                    frame_mode = "CLOCKED"

                    initialize_recordprocessing()
                    
                elif buffer.startswith( "OUTER CLOCK" ):
                    try:
                        frame_outerclock = int(buffer[11:])
                    except Exception as e:
                        print( buffer, e )
                        self.errorflag.value += 1
                        self.textqueue.put( 'Error: ' + buffer )

                
                # ---------------------------------------
                elif buffer.startswith( "GATE START" ):
                    #print( buffer )
                    frame_mode = "GATE"
                    initialize_recordprocessing()

                # ---------------------------------------
                elif buffer.startswith( "ADC" ) or buffer.startswith( "CHIPTEMPERATURE" ):
                    if busyflag.value:
                        dataqueue_put( buffer )
                    else:
                        self.textqueue.put( buffer )
                        if self.monitorqueue:
                            self.monitorqueue.put( buffer.strip() + '\n' )
                        
                elif buffer.startswith( "COMPLETE" ):

                    if accumulate:
                        enqueueaccumulators()                        

                    # ---------------------
                    if debug:
                        print( 'lccd clearing busyflag' )
                    busyflag.value = 0
                    
                    
                elif buffer.startswith( "coefficient" ):

                    # this also generates self.xdata
                    self.parsecoefficients( buffer )
                    print( self.coefficients )
                    print( self.xdata )
                    
                elif buffer.startswith( "Identifier" ):

                    self.identifier = buffer.split(maxsplit=1)

                # --------------------------------
                # Receive Ascii Formatted Text
                elif ( buffer[0] == '#' ):
                    print( buffer[1:] )

                elif ( buffer.startswith( "Error:") ):

                    #print( buffer );
                    self.errorflag.value += 1
                    
                    self.textqueue.put( buffer )
                    if self.monitorqueue:
                        self.monitorqueue.put( buffer.strip() + '\n' )

                else:
                    
                    self.textqueue.put( buffer )
                    if self.monitorqueue:
                        self.monitorqueue.put( buffer.strip() + '\n' )

        print( "reader exit" )
        sys.exit()

    # --------------------------------------------
    def checkerrors( self ):

        counter = self.errorflag.value
        self.errorflag.value = 0

        return counter
        
    def read_all_( self ):
        resp = []
        while not self.textqueue.empty():
            resp.append( self.textqueue.get() )  
        return resp

    def read_to_done_( self ):
        resp = []
        while True:
            line = self.textqueue.get()
            if self.debug:
                print( "read_to_done_, got:", line )
            if line.startswith( "DONE"):
                break
            resp.append( line )
        return resp

    def read( self ):

        while self.textqueue.empty() and not input_ready():
            sleep(.1)

        return self.read_to_done_()

    def read_nowait( self ):
        return self.read_all_()

    def write( self, buffer ):
        self.ser.write( buffer.encode() )

    def writeread( self, line ):

        self.write( line + '\n' )
    
        response = self.read()
        for line in response:
            print( 'response: ', line )

        return response
        
    def clear( self ):
        
        while not self.textqueue.empty():
            self.textqueue.get()
            
        while not self.dataqueue.empty():
            self.dataqueue.get()
            
    def close( self, ignored=None ):

        self.flag.value = 0
        sleep( 0.1 )
        #self.ser.reset_input_buffer()
        #self.ser.close()

        self.readerthread.terminate()        
        self.readerthread.join( )

        if self.monitorthread:
            self.monitorthread.terminate()
            self.monitorthread.join()

        if self.GraphicsWindow:
            self.GraphicsWindow.close()

    # =============================================================================================
    def savetofile( self, file, timestamp=None, comments = None, write_ascii=True, framesetindex = None, records = None ):

        def formattedwrites( file, keys, vals, exclude=None ):
            for key,val in zip( keys,vals ):

                if exclude is not None:
                    if key in exclude:
                        continue
                    
                if type(val) in [ float, int ] :
                    file.write( '# ' + key + ' = ' + str(val) + '\n' )

                elif type(val) in [ str ] :
                    file.write( '# ' + key + ' = "' + str(val) + '"\n' )

                elif type(val) in [ list ] :
                    if not len(val):
                        file.write( '# ' + key + ' = []\n' )
                    else:
                        values = val
                        if type(values[0]) in [ float, int ] :
                            file.write( '# ' + key + ' = [ ' + ', '.join( [ str(v) for v in values ] ) + ' ]\n'  )
                        elif type(values[0]) in [ str ] :
                            file.write( '# ' + key + ' = [ "' + '", "'.join( [ str(v) for v in values ] ) + '" ]\n'  )

        # -------------------------
        
        newfile = False

        if timestamp is None:
            timestamp = datetime.now()
        
        if type(file) is str:

            if '%' in file:
                try:
                    exec( "file="+file, self.__dict__, globals() )
                    print(file)
                except Exception as e:
                    print( e )
                    return False

            if not file.endswith( self.filesuffix ):
                file += "." + timestamp.strftime('%Y%m%d.%H%M%S.%f') + self.filesuffix
                
            try:
                file = open(file,"x")
                print( 'saveto', file )
                newfile = True
            except Exception as e:
                print( "open " + file )
                print( e )
                return False
            
        # ------------------------------------------------
            
        file.write( '# LCCDController.py version %s\n'% __version__ )

        file.write( '# ' + timestamp.strftime('%Y-%m-%d %H:%M:%S.%f') + '\n' )

        if self.identifier is not None:
            s = self.identifier
            s = s.replace('\r', '')
            s = s.strip()
            file.write( '# identifier = "' + s + '"\n' )

        # Save all of the numerical and string values in the header portion of the class
        keys, vals = zip(*self.__dict__.items())
        formattedwrites( file, keys, vals, exclude=['identifier','filesuffix'] )
        
        if comments is not None:
            if type(comments) not in [list,tuple]:
                comments = [comments]

            for c in comments:
                file.write( '# comment = "' + c + '"\n' )

    
        file.write( '# header end\n' )

        if records is None:
            records = []
            while not self.dataqueue.empty():
                record = self.dataqueue.get()
                records.append(record )
                if self.dataqueue.empty():
                    sleep(0.2)

        print( 'lccd writing', len(records), 'records' )
            
        wroterecord = False
        for record in records:

            # If starting the next record, start with two blank lines
            if wroterecord:
                file.write( "\n" )
                file.write( "\n" )
                wroterecord = False

            # ----------------------------
            if type(record) is str:
                file.write( '# ' + record.strip() + '\n' )
                    
            else:
                ycols, \
                    frame_shutter, frame_interval, frame_clock, \
                    frame_counter, frame_outercounter, \
                    frame_frames, frame_sets, frame_every, \
                    frame_mode, frame_elapsed, \
                    accumulate, timestamp = record

                vals = record[1:]
                
                keys = "SHUTTER", "INTERVAL", "CLOCK", \
                    "COUNTER", "OUTERCOUNTER", \
                    "FRAMES", "SETS", "EVERY", \
                    "MODE", "ELAPSED", \
                    "ACCUMULATE", "TIMESTAMP"

                formattedwrites( file, keys, vals )

                if write_ascii:
                    for n, ycol in enumerate(ycols):
                        file.write( "# DATA ASCII %d COL %d\n"%(len(ycol),n) )
                        for y in ycol:
                            file.write( '%.8f\n'%(y) );
                        file.write( "# END DATA\n" )
                else:
                    for n, ycol in enumerate(ycols):
                        file.write( "# DATA %s %d COL %d\n"%(type(ycol[0]), len(ycol),n) )
                    file.write( "# END DATA\n" )                    
                
                wroterecord = True

        if newfile:
            file.close()
                  
    # =====================================================================
    def commandlineprocessor( self, line, fileprefix=None ):

        # String ubstitutions
        if '%(' in line:
            parts = list( split_bracketed( line ) )
            for n,p in enumerate(parts):
                if p.startswith('"') and '"%(' in p:
                    exec( "res="+p, self.__dict__, globals() )
                    parts[n] =res                    
            line = ' '.join( parts )
            print( "command with string substitution:", line )
        
        if self.monitorqueue:
            self.monitorqueue.put( 'command: ' + line + '\n')

            
        
        if line in [ 'h', 'help' ]:
            
            self.write( 'help\n' )           
            response = self.read()

            print( "  " )

            print( "Commands implemented in the CLI/host computer:" )
            print( "   h|help                      - produces this help text" )
            print( "" )
            print( "   accumulator on | off        - turn frame-wise accumulator on/off in the reader thread" )
            print( "   baseline on | off           - turn basline correction on/off" )
            print( "" )
            print( "   add init                    - initialize the local accumulators" )
            print( "   add                         - get and add the contents of the data queue to the local accumulators" )
            print( "   add push                    - push the local accumulators onto the dataqueue for the save command" )
            print( "" )
            print( "   save fileprefix comments... - save data to diskfile" )
            print( "   wait                        - wait for completion of the active frameset" )
            print( "" )
            print( "   @filespec                   - read and execute commands from a file" )
            print( "      in batch files,';' is a command separator, use \; to escape for shell commands" )
            print( "" )
            print( "   !command                    - execute shell command" )
            print( "" )
            print( "   a = 3                       - '=' causes evaluation as python" )
            print( "   = python statement          -  pass to python interprator" )
            print( "      these commands have access to local() and class name spaces" )
            print( "" )
            print( "   q[uit]                      - exit the cli program" )
            
        elif line.startswith( '#' ):
            print( "rcvd comment line" )
            print( line )

        elif line.startswith('wait'):

            pars = line.split()
            if len(pars) > 3:
                try:
                    if not self.wait( float(pars[2] ), bool(pars[3]) ):
                        return False
                except Exception as e:
                    print( e )
                    return False
            elif len(pars) > 2:
                try:
                    if not self.wait( float(pars[2]), True ):
                        return False
                except Exception as e:
                    print( e )
                    return False
            else:
                if not self.wait( interruptible=True ):
                    return False

        elif line.startswith('clear'):
            self.clear()
            
        elif line.startswith('save'):

            pars = line.split( maxsplit = 2 )

            print( pars )

            if len( pars ) == 1:
                print( 'need filespec [comments]' )

            elif len(pars) == 2:
            
                self.savetofile( pars[1], write_ascii=True )

            elif len(pars) == 3:
            
                self.savetofile( pars[1], comments = pars[2], write_ascii=True )

        # Local accumulator in the CLI
        elif line.startswith('add') or line.startswith( "local accumulat" ):

            # Initialize the local accumulator
            if 'init' in line:
                return self.accumulators.initialize()
                
            if 'graph' in line:
                return self.accumulators.graph()
            
            # Put it back on the queue
            if 'push' in line:
                return self.accumulators.push()

            # Normal acculumulator add
            return self.accumulators.pull()

        # Accumulate in the reader thread
        elif line.startswith( 'accumulat' ):
            if 'off' in line:
                self.accumulatorflag.value = 0
            else:
                self.accumulatorflag.value = 1

        elif line.startswith( 'baseline' ):
            if 'off' in line:
                self.baselineflag.value = 0
            else:
                self.baselineflag.value = 1

        elif line.startswith('@'):

            batchfile = line[1:].strip()
            
            try:
                with open( batchfile, 'r' ) as f:
                    script = f.readlines()
            except Exception as e:
                print(e)
                return False

            for line in script:
                line = line.strip()
                status = True

                # Protect the semicolons
                if ';' in line:
                    line = line.replace( '\;', '{\semicolon}' )
                    
                for line_ in line.split(';'):

                    # Restore the semicolons
                    line_ = line_.replace( '{\semicolon}', ';' )
                    line_ = line_.strip()
                    print( "command:", line_ )
                    
                    if not self.commandlineprocessor( line_, fileprefix ):
                        status = False
                        break
                if not status:
                    break

        elif line.startswith('!'):
            result = os.popen( line[1:] ).read()
            if result:
                print( result )

        elif '=' in line:

            if line == '=':
                for key, val in globals().items():
                    print( "global ", key, val )
                for key, val in locals().items():
                    print( "local ",key, val )

            elif line.startswith( '=' ):
                try:
                    exec( line[1:].strip(), self.__dict__, globals() )
                except Exception as e:
                    print( e )
                    return False
                
            else:
                try:
                    exec( line, self.__dict__, globals() )
                except Exception as e:
                    print( e )
                    return False

        elif line.startswith('for'):
            
            loopspec, line_ = line.split(':',maxsplit=1)
            
            parts = list( split_bracketed( loopspec ) )
            if len( parts ) != 4 or parts[2] != 'in':
                print(  'loopspec not valid', loopspec )
                return False

            exec( "loopvalues = list(" + parts[3] + ")", self.__dict__, globals() )
            print( loopvalues )

            for v in loopvalues:

                exec( parts[1] + " = " + str(v), self.__dict__, globals() )

                for line__ in line_.split(';'):

                    line__ = line__.strip()
                    
                    if line__.startswith( '"' ):
                        exec( "line___ = " + line__, self.__dict__, globals() )
                        print( "command:", line___ )
                        if not self.commandlineprocessor( line___, fileprefix ):
                            return False                   
                    else:
                        print( "command:", line__ )
                        if not self.commandlineprocessor( line__, fileprefix ):
                            return False

            
        elif line.startswith( 'parse' ):
            try:
                print( list( split_bracketed( line ) ) )
            except Exception as e:
                print( e )

        elif line is not None and len(line) > 0:

            self.write( line + '\n' )
            
            response = self.read()
            for line in response:
                print( 'response: ', line )

        else:
            response = self.read_nowait()
            if len(response) :
                for line in response:
                    print( '>response: ', line )

        if self.checkerrors():
            print( 'checkerrors found errors' )
            return False
        
        return True
        
        
    def commandloop( self, name="SerialMonitor", fileprefix=None ):
        
        while self.flag.value:

            line = input( name + ':' )

            if line.lower() in ['exit', 'quit', 'q' ]:
                break

            count = self.checkerrors()
            if count:
                print( "rcvd %d errors previous to this command"%(count) )
                        
            self.commandlineprocessor( line, fileprefix )

            count = self.checkerrors()
            if count:
                print( "Error detected", count )            

        return
                    
# =========================================================================================================

if __name__ == "__main__":

    import argparse

    try:
        import readline
    except:
        pass

    import atexit
    import signal
    
    if platform.system() == 'Linux':
        ser0_default = '/dev/ttyACM0'
        ser1_default = '/dev/ttyACM1'
    elif platform.system() == 'Windows':
        ser0_default = 'COM1:'
        ser1_default = 'COM2:'

    # ---------------------------------------------------------
    def SignalHandler(signal, frame):
        print('Ctrl-C')

        serialdevice.close()
        if dataport is not None:
            dataport.close()
            
        print('Exit')
        sys.exit(0)
    
    # ---------------------------------------------------------
    class ExplicitDefaultsHelpFormatter(argparse.ArgumentDefaultsHelpFormatter):
        def _get_help_string(self, action):
            if action.default in (None, False):
                return action.help
            return super()._get_help_string(action)
    
    parser = argparse.ArgumentParser( description='LCCD Controler Monitor/Cli',
                                      formatter_class=ExplicitDefaultsHelpFormatter )

    parser.add_argument( 'ports', default=[ser0_default], nargs='*',
                         help = 'one or more serial or com ports,' +
                         ' the first is the control port, others are readonly' )

    parser.add_argument( '--historyfile', default = 'tcd1304rev2controller.history', help='history file for the command line interface' )
    parser.add_argument( '--nohistoryfile' )
    
    parser.add_argument( '--pixels', action = 'store_true', help='graph pixel indices on x-axis' )
    
    parser.add_argument( '--guiwindow', action = 'store_true', help='gui version of the graphical display' )
    
    parser.add_argument( '--loggingwindow', action = 'store_true', help='display transactions in a scrolling text window' )

    parser.add_argument( '--raw', action = 'store_true', help='graph raw binary values' )

    parser.add_argument( '--debug', action = 'store_true' )

    # for datafile reading
    parser.add_argument( '--xrange', nargs=2, type=float )
    parser.add_argument( '--yrange', nargs=2, type=float )    
    parser.add_argument( '--dump', action = 'store_true' )
    parser.add_argument( '--graph', action = 'store_true' )
    parser.add_argument( '--frame', type=int )
    parser.add_argument( '--x', help = 'xdata or python rexpression' )
    parser.add_argument( '--y', help = 'ydata or python expression' )
    parser.add_argument( '--output' )
    
    args = parser.parse_args()

    # ----------------------------------------------------------
    if os.path.isfile( args.ports[0] ):
        print( 'reading file', args.ports[0] )
        dataobject = LCCDDATA( args.ports[0] )
        if args.dump:
            dataobject.dump()
        if args.graph:
            xdata = dataobject.xdata
            if args.x is not None:
                exec( 'xdata = '+args.x )
            if args.frame is not None:
                frame = dataobject.frames[args.frame]
                ydata = frame.data
                if args.y is not None:
                    exec( 'ydata = '+args.y )
                plt.plot( xdata, ydata )
            else:
                for n, frame in enumerate(dataobject.frames):
                    ydata = frame.data
                    if args.y is not None:
                        exec( 'ydata = '+args.y )
                    plt.plot( xdata, ydata,label=str(frame.offset))
                plt.legend(title=dataobject.mode+', t(secs)')

            if args.output is not None:
                plt.savefig( args.output )
            else:
                plt.show()
        
        quit()
        
    # ---------------------------------------------------------
    serialdevice = LCCDCONTROLLER( args.ports[0], monitor=args.loggingwindow, graph_by_pixels=args.pixels, gui=args.guiwindow, debug=args.debug )
    dataport = None
    
    if len(args.ports) > 1:
        dataport = LCCDCONTROLLER( args.ports[1] )

    # ---------------------------------------------------------
    if not args.nohistoryfile:
        try:
            readline.read_history_file(args.historyfile)
        except Exception as e:
            print('historyfile: ', e)
            print('continuing')
            
        atexit.register(readline.write_history_file, args.historyfile)

    signal.signal(signal.SIGINT, SignalHandler)
    
    # ---------------------------------------------------------

    sleep(1)
    print( "" )
    print( versionstring )
    
    serialdevice.commandloop( name="LCCD", fileprefix=None )

    serialdevice.close()
