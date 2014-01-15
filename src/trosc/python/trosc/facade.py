import sys
import ctypes
import sysv_ipc as ipc

class Command( ctypes.Structure ):
  _fields_ = [ ( "steering", ctypes.c_float ), 
               ( "acceleration", ctypes.c_float ),
               ( "brake", ctypes.c_float ),
               ( "gear", ctypes.c_int ) ]

class Status( ctypes.Structure ):
  _fields_ = [ ( "rpm", ctypes.c_float ),
               ( "gear", ctypes.c_int ),
               ( "gear_ratio", ctypes.c_float ),
               ( "lower_gear_ratio", ctypes.c_float ),
               ( "max_rpm", ctypes.c_float ),
               ( "wheel_radius", ctypes.c_float ),
               ( "track_yaw", ctypes.c_float ),
               ( "track_distance", ctypes.c_float ),
               ( "track_curvature", ctypes.c_float ),
               ( "track_width", ctypes.c_float ),
               ( "next_curvature", ctypes.c_float ),
               ( "next_distance", ctypes.c_float ),
               ( "speed", ctypes.c_float ),
               ( "yaw", ctypes.c_float ),
               ( "x", ctypes.c_float ),
               ( "y", ctypes.c_float )]

class Obstacle( ctypes.Structure ):
  _fields_ = [ ( "id", ctypes.c_byte ),
               ( "x", ctypes.c_float ),
               ( "y", ctypes.c_float ),
               ( "vX", ctypes.c_float ),
               ( "vY", ctypes.c_float ),
               ( "width", ctypes.c_float ),
               ( "height", ctypes.c_float ) ]

class Buffer( ctypes.Structure ):
  _fields_ = [ ( "command", Command ), 
               ( "status", Status ),
               ( "n_obstacles", ctypes.c_int ),
               ( "obstacles", Obstacle * 100 ) ]

class Facade( object ):
  def __init__( self, key = 5555 ):
    libc = ctypes.CDLL( "libc.so.6" )
    libc.shmat.restype = ctypes.POINTER( Buffer )

    segment_id = libc.shmget( key, 0, 0 )
    self.buffer = libc.shmat( segment_id, 0, 0 )
    self.s = ipc.Semaphore( key )
  
  def __get_status( self ):
    self.s.acquire()
    b = Buffer.from_buffer_copy( self.buffer.contents )
    self.s.release()
    return b.status

  def __get_obstacles( self ):
    self.s.acquire()
    b = Buffer.from_buffer_copy( self.buffer.contents )
    self.s.release()
    return b.n_obstacles, b.obstacles

  def __set_command( self, command ):
    self.s.acquire()
    self.buffer.contents.command = command
    self.s.release()

  status = property( __get_status )
  obstacles = property( __get_obstacles )
  command = property( None, __set_command )
