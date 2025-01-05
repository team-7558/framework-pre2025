import subprocess

subprocess.run([
    "ssh", f"admin@10.75.58.2",
    "/usr/local/natinst/etc/init.d/systemWebServer", "start"
])