from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("usd_path", type=str)
parser.add_argument("output_path", type=str)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from pxr import Usd

stage = Usd.Stage.Open(args.usd_path)
stage.GetRootLayer().Export(args.output_path)
app.close()
