import os
from pprint import pprint
from Gaudi.Configuration import *

from Configurables import ActsAlg, GeoSvc, ActsGeoSvc
algList = []

geosvc = GeoSvc("GeoSvc")
geosvc.detectors = [f"{os.environ['OPENDATADETECTOR']}/xml/OpenDataDetector.xml"]
# using this instead of CLIC_o3_v14 because it is much faster to instantiate
#geosvc.detectors = [os.environ["LCGEO"]+"/CLIC/compact/CLIC_o2_v04/CLIC_o2_v04.xml"]
geosvc.EnableGeant4Geo = False

actsgeosvc = ActsGeoSvc('ActsGeoSvc')
actsgeosvc.GeoSvcName = geosvc.name()
actsgeosvc.debugGeometry = True
actsgeosvc.outputFileName = "MyObjFile"

actsalgo = ActsAlg('ActsAlg')
actsalgo.trackFindingAlgorithm = 'ActsTrackFindingAlg'
actsalgo.trackFittingAlgorithm = 'ActsTrackFittingAlg'
algList.append(actsalgo)

from Configurables import ApplicationMgr
ApplicationMgr( TopAlg = algList,
                EvtSel = 'NONE',
                EvtMax = 2,
                ExtSvc = [geosvc, actsgeosvc],
                OutputLevel=INFO
              )
