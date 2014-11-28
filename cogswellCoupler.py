__author__ = 'Sergio Sykes'

"""
Usage:
Add this script to your maya/scripts directory

In maya, from the Python Command line, enter:
import cogswellCoupler; reload(cogswellCoupler).GUI()

This script assumes your rig skeleton is parented directly under your world/main controller
"""

import pymel.core as pmc


class CouplerApp():
    """
    This coupler detects joints using name prefixes, 
    you can use either my default settings in __init__ or pass your own
    """

    def __init__(self, bindPrefix='jnt', rigPrefix='rig'):
        """
        
        """
        self._bindPrefix = bindPrefix
        self._rigPrefix = rigPrefix
        self.mainWindow = None

    def Draw(self, windowName):
        """
        Creates the UI for using script
        """

        if pmc.window(windowName, exists=True):
            pmc.deleteUI(windowName, window=True)

        self.mainWindow = pmc.window(windowName, title='Advanced Rigging Coupler (Basic)', width=250)
        pmc.columnLayout(adjustableColumn=True, rowSpacing=5)
        pmc.text(label='Select Rig\'s main control', align='center')
        pmc.button(label='Connect', backgroundColor=(1, 0.5, 0.5), command=pmc.Callback(self._callback, False))
        pmc.button(label='Disconnect', backgroundColor=(0.5, 0.5, 1), command=pmc.Callback(self._callback, True))
        self.mainWindow.show()

    def _callback(self, doDisconnect):
        """
        Handles actual rig connection.
        """

        rootControl = pmc.ls(selection=True)
        joints = [i for i in rootControl[0].getChildren(ad=True, type='joint') if i.startswith(self._rigPrefix)]

        for jnt in joints:
            if doDisconnect:
                skinJoint = jnt.replace(self._rigPrefix, self._bindPrefix, 1)
                if not pmc.objExists(skinJoint):
                    continue

                constraintNodes = set(pmc.listConnections(skinJoint, type='constraint', d=False, s=True))
                pmc.delete(list(constraintNodes))

            else:
                #Find matching jnt in skeleton, if available
                skinJoint = jnt.replace(self._rigPrefix, self._bindPrefix, 1)
                if not pmc.objExists(skinJoint):
                    continue

                pmc.pointConstraint(jnt, skinJoint, maintainOffset=False)
                orient = pmc.orientConstraint(jnt, skinJoint, maintainOffset=False)
                orient.interpType.set(2)    #Interplotion 2 = shortest (to avoid flipping)

        #After disconnect, revert skeleton to bind pose
        if doDisconnect:
            skinRoot = joints[0]
            dagPoses = pmc.listConnections(skinRoot, type='dagPose', d=True, s=False)
            if len(dagPoses):
                pmc.dagPose(skinRoot, restore=True, g=True, bindPose=True, name=dagPoses[0])
            print 'COGSWELL COUPLER :: Rig Disconnected'
        else:
            print 'COGSWELL COUPLER :: Rig Connected'


##MAIN EXECUTION
def GUI():
    global MAIN_WINDOW

    MAIN_WINDOW = CouplerApp()
    MAIN_WINDOW.Draw('AdvancedRiggingCoupler')

    #done