import glob
import os
from invoke import task, Collection, call

THISDIR = os.path.dirname(os.path.realpath(__file__))
MODULE_NAME = os.path.basename(THISDIR)

def get_submodules(c):
    """
    return repository submodule names
    """
    submodules = []
    with c.cd(THISDIR):
        result = c.run("git submodule status", hide=True)
        for line in result.stdout.splitlines():
            submodules.append(line.split()[1])
    return submodules

def get_iname_tag(image_name):
    """
    return tuple with image name and tag
    """
    if ":" in image_name:
        iname, tag = image_name.split(":")
    else:
        iname, tag = image_name, "latest"
    return iname, tag

@task
def init(c):
    """
    Init submodules.
    """
    print("init submodules")
    with c.cd(THISDIR):
        c.run("git submodule init", hide=True)

@task(init)
def clone(c):
    """
    Clone this repository submodules.
    """
    submodules = get_submodules(c)
    with c.cd(THISDIR):
        for sub in submodules:
            c.run("git submodule update --init --recursive %s" %sub)

@task(
    help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image",
          'ros_distro': "ROS distro to use (Available [foxy, galactic])"}
)
def build_env(c, nocache=False, pull=False, ros_distro="foxy", image_name=MODULE_NAME):
    """
    Create Docker build environment.
    """
    iname, tag = get_iname_tag(image_name)

    args = []
    args.append("--build-arg UID=$(id -u)")
    args.append("--build-arg GID=$(id -g)")
    args.append("--build-arg ROS_DISTRO=%s" % ros_distro)
    args.append("--build-arg PACKAGE_NAME=%s" % iname)
    args.append("-f Dockerfile.build_env")
    args.append("-t %s_build:%s" % (iname, tag))
    if nocache:
        args.append("--no-cache")
    elif pull:
        args.append("--pull")
    with c.cd(THISDIR):
        c.run("docker build %s ." % " ".join(args))

@task(
    help={'reallyclean': "remove & reload all submodules"}
)
def clean(c, reallyclean=False):
    """
    Clean workspace.
    """
    with c.cd(THISDIR):
        if reallyclean:
            c.run("git submodule deinit -f --all")
            clone(c)
        else:
            c.run("git submodule foreach git clean -xdf")
            c.run("git submodule foreach git checkout .")
        c.run("git clean -xdf")

@task(
    help={'out_dir': "output directory for the generated deb files",
          'ros_distro': "ROS distro to use (Available [foxy, galactic])"}
)
def create_deb_package(c, out_dir="../bin/", ros_distro="foxy", image_name=MODULE_NAME):
    """
    Build debian package
    """
    iname, tag = get_iname_tag(image_name)
    c.run("ROS_DISTRO={0} PACKAGE_NAME={1} PACKAGE_VERSION={2} ./build.sh {3}"
          .format(ros_distro, iname, tag, out_dir))

@task(help={'nocache': "do not use cache when building the image",
          'pull': "always attempt to pull a newer version of the image",
          'ros_distro': "ROS distro to use (Available [foxy, galactic])",
          'image_name': "name of output docker image"}
)
def build_docker(c, nocache=False, pull=False, ros_distro="foxy", image_name=MODULE_NAME):
    """
    Build Docker image of this component
    """
    col = Collection()
    col.add_task(build_env)
    col['build_env'](c, nocache=nocache, pull=pull, ros_distro=ros_distro, image_name=image_name)

    iname, tag = get_iname_tag(image_name)
    args = []
    args.append("--build-arg UID=$(id -u)")
    args.append("--build-arg GID=$(id -g)")
    args.append("--build-arg ROS_DISTRO=%s" % ros_distro)
    args.append("--build-arg PACKAGE_NAME=%s" % iname)
    args.append("--build-arg FROM_IMAGE=%s_build:%s" % (iname, tag))
    args.append("-f Dockerfile")
    args.append("-t %s:%s" % (iname, tag))
    if nocache:
        args.append("--no-cache")
    elif pull:
        args.append("--pull")
    with c.cd(THISDIR):
        print("docker build %s ." % " ".join(args))
        c.run("docker build %s ." % " ".join(args))

