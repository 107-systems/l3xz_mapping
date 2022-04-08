#!/bin/bash
rm -r dockerid.id
docker run -it --cidfile dockerid.id -p 8000:8000 -p 9090:9090 l3x_mapping /bin/bash
