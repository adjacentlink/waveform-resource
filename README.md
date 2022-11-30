waveform-resource
==

Waveform Resource is an event driven framework for writing
client/server service plugin daemons with an application context
providing events for registered UDP and TCP communication channels,
interval timers, and file descriptors.

The `waveform-resourced` application uses an epoll event loop to
dispatch all events and provides logging, daemonize, and pid file
support. `waveform-resourced` takes as a parameter the full module
path of the Waveform Resource application plugin along with additional
options such as configuration file, pid file, and daemonize flag.

```
$ waveform-resourced waveform_resource.plugins.my_service.app.Plugin  \
    --config-file my-service-app.xml \
    --log-file /path/to/var/log/waveform-resourced-app.log \
    --pid-file /path/to/var/log/waveform-resourced-app.pid \
    --daemonize  \
    --log-level debug
```

See `pydoc waveform_resource` for more information.
