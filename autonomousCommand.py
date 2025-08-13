from commands2 import Command
from drivetrain import DriveTrain


class WaitXSeconds(Command):
   def __init__(self, drivetrain: DriveTrain, seconds: int):
       self.drivetrain = drivetrain
       self.seconds = seconds
       self.counter = 50 * seconds
       self.addRequirements(self.drivetrain)
       print ("Waiting for ", seconds,  "  Command Initialized")

   def initialize(self):
       super().initialize()


   def execute(self):
       self.drivetrain.drive_teleop(0.0, 0.0)
       self.counter = self.counter - 1


   def isFinished(self) -> bool:
       if self.counter <= 0:
           return True
       else:
           return False
  
   def end(self, interrupted: bool):
       self.drivetrain.drive_teleop(0.0, 0.0)