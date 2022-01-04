#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------

def get_number_range(str):
    #extract range
    range = str.split(':')
    if len(range) == 2:
        try:
            range[0] = float(range[0]) 
            range[1] = float(range[1]) 
            return range;
        except ValueError:
            return None
    return None

def get_numberlist(str):
    #extract list
    list = str.split(',')
    if len(list) > 1:
        return list
    else:
        return None

def is_valid_number_range(va,vb,vmin,vmax):
    if not ( va<vb and va>=vmin and vb<=vmax ):
        return False
    return True;

def is_valid_numberlist(vlist,min,max):    
    for v in vlist:
        if not is_number(v):
            return False
        if not (v<= max and v>=min):
            return False

def is_positive_number(v):
    return v is not None and is_number(v) and (float(v) > 0)

def is_number(v):
    try:
        float(v)
        return True
    except ValueError:
        return False