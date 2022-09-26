#!/usr/bin/env python3

import os
import capnp

capnp.add_import_hook(['../src/capnp'])

import imu_capnp as eCalImu

msg = eCalImu.Imu.new_message()