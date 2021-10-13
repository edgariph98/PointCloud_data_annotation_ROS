import os
import rospy
from argparse import ArgumentParser
import rospkg
from qt_gui.plugin import Plugin
from components import MainApp
class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Plugin Object Name
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Adding Main App Widget 
        context.add_widget(MainApp())