import pyopencl as cl
import numpy as np
import belly

class Program( object ):
  context = cl.create_some_context()
  queue = cl.CommandQueue( context )
  programs = {}

  def __init__( self ):
    self.context = Program.context 
    self.queue = Program.queue

  def loadProgram( self, filename ):
    if filename in Program.programs:
      return Program.programs[filename]
    f = open( filename, 'r' )
    fstr = "".join( f.readlines() )
    program = cl.Program( self.context, fstr ).build( "-I %s/opencl/" % belly.BASE_DIR )
    Program.programs[filename] = program
    return program
