"""Desktop-managed communication stays on this laptop; no web/LAN application endpoint."""
import os


def local_environment(domain,base=None):
    env=dict(os.environ if base is None else base)
    env['ROS_DOMAIN_ID']=str(domain)
    env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    env['ROS_LOCALHOST_ONLY']='1'  # Also covers older installed RMW configurations.
    env.pop('ROS_STATIC_PEERS',None)
    env['GZ_IP']='127.0.0.1'
    env['GZ_PARTITION']=f'dual_scara_desktop_{os.getuid()}_{domain}'
    return env


def configure_process(domain):
    env=local_environment(domain)
    for name in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_LOCALHOST_ONLY','GZ_IP','GZ_PARTITION']:
        os.environ[name]=env[name]
    os.environ.pop('ROS_STATIC_PEERS',None)
