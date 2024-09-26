#!/bin/bash

uglifyjs -c -- js/midi.js > js/midi.min.js
uglifyjs -c -- js/constants.js > js/constants.min.js

uglifycss css/config.css > css/config.min.css
  
