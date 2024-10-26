from os.path import join
Import("env")

env.Replace(
    PROGSUFFIX=".bin"
)
env.Replace(
    UPLOADCMD=f"./tools/upload.sh $PROGPATH"
)