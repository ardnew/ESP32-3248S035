#import os
#import traceback
#from shutil import copy
#
#try:
#    Import("env")
#except:
#    traceback.print_exc()
#    try:
#        env = DefaultEnvironment()
#    except:
#        traceback.print_exc()
#
#print(env)
#
#board = 'boards/ESP32-3248S035C.json'
#
#shutil.copy(board, pio_boards_dir)
