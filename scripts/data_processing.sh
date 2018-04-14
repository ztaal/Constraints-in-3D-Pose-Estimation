#!/bin/bash
path="$1"

sed -i '1d' $path/*
sed -i '1d' $path/*
sed -e s/T-rex,//g -i $path/*
sed -e s/chef,//g -i $path/*
sed -e s/chicken,//g -i $path/*
sed -e s/parasaurolophus,//g -i $path/*
sed -e s/rhino,//g -i $path/*
