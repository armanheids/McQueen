#!/bin/bash
P=/home/claude/tools/prism-4.8.1-linux64-x86
export LD_LIBRARY_PATH=$P/lib:$LD_LIBRARY_PATH
exec java -Xmx1g -Djava.awt.headless=true -Djava.library.path=$P/lib -cp "$P/lib/prism.jar:$P/lib/*:/home/claude/mqg_exp/prism_driver" MqgPrismServer "$@"
