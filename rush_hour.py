import sys
from z3 import *

file1 = open(sys.argv[1], 'r')
count = 0
vx = []
vy = []
ver_init = []
hx = []
hy = []
hor_init = []
mx = []
my = []
mine_pos = []
for line in file1:
    count += 1
    line = line.strip()
    l = line.split(',')
    if count == 1:
        n = int(l[0])
        lim = int(l[1])
        rx = [Int('rx_%i' %i) for i in range(lim + 1)]
        ry = [Int('ry_%i' %i) for i in range(lim + 1)]
    elif count == 2:
        red_init = (int(l[0]),int(l[1]))
    elif l[0] == '0':
        j = len(vx)
        vx.append([Int('vx_%s_%s' %(j,i)) for i in range(lim + 1)])
        vy.append([Int('vy_%s_%s' %(j,i)) for i in range(lim + 1)])
        ver_init.append((int(l[1]),int(l[2])))
    elif l[0] == '1':
        j = len(hx)
        hx.append([Int('hx_%s_%s' %(j,i)) for i in range(lim + 1)])
        hy.append([Int('hy_%s_%s' %(j,i)) for i in range(lim + 1)])
        hor_init.append((int(l[1]), int(l[2])))
    else:
        j = len(mx)
        mx.append(Int('mx_%s' %j))
        my.append(Int('my_%s' %j))
        mine_pos.append((int(l[1]), int(l[2])))
num_ver = len(vx)
num_hor = len(hx)
num_mine = len(mx)

ver_lim = [ And(0 <= vx[i][j], vx[i][j] < n, 0 <= vy[i][j], vy[i][j] < n-1) for i in range(num_ver) for j in range(lim + 1) ]
hor_lim = [ And(0 <= hx[i][j], hx[i][j] < n-1, 0 <= hy[i][j], hy[i][j] < n) for i in range(num_hor) for j in range(lim + 1) ]
red_lim = [ And(0 <= rx[i], rx[i] < n-1, 0 <= ry[i], ry[i] < n) for i in range(lim + 1) ]

limits = ver_lim + hor_lim + red_lim

init_cond_h = [ And(hx[i][0] == hor_init[i][1], hy[i][0] == hor_init[i][0]) for i in range(num_hor) ]
init_cond_v = [ And(vx[i][0] == ver_init[i][1], vy[i][0] == ver_init[i][0]) for i in range(num_ver) ]
init_cond_r = [ And(rx[0] == red_init[1], ry[0] == red_init[0]) ]
mine_pos = [ And(mx[i] == mine_pos[i][1], my[i] == mine_pos[i][0]) for i in range(num_mine) ]

init = init_cond_r + init_cond_h + init_cond_v + mine_pos

goal = [Or([(rx[i] == n-2) for i in range(lim + 1)])]

prevent_mine_h = [ Not(And(Or(hx[i][k] + 1 == mx[j], hx[i][k] == mx[j]), hy[i][k] == my[j])) for i in range(num_hor) for j in range(num_mine) for k in range(lim + 1) ]
prevent_mine_v = [ Not(And(Or(vy[i][k] + 1 == my[j], vy[i][k] == my[j]), vx[i][k] == mx[j])) for i in range(num_ver) for j in range(num_mine) for k in range(lim + 1) ]
prevent_mine_r = [ Not(And(Or(rx[k] + 1 == mx[j], rx[k] == mx[j]), ry[k] == my[j])) for j in range(num_mine) for k in range(lim + 1) ]
prevent_coll_h_v = [ Not(Or(And(hx[i][k] == vx[j][k], hy[i][k] == vy[j][k]), And(hx[i][k] == vx[j][k], hy[i][k] == vy[j][k] + 1), And(hx[i][k] == vx[j][k] - 1, hy[i][k] == vy[j][k]), And(hx[i][k] == vx[j][k] - 1, hy[i][k] == vy[j][k] + 1))) for i in range(num_hor) for j in range(num_ver) for k in range(lim + 1) ]
prevent_coll_r_v = [ Not(Or(And(rx[k] == vx[j][k], ry[k] == vy[j][k]), And(rx[k] == vx[j][k], ry[k] == vy[j][k] + 1), And(rx[k] == vx[j][k] - 1, ry[k] == vy[j][k]), And(rx[k] == vx[j][k] - 1, ry[k] == vy[j][k] + 1))) for j in range(num_ver) for k in range(lim + 1) ]
prevent_coll_r_h = [ Not(And(Or(rx[k] == hx[i][k], rx[k] == hx[i][k] - 1, rx[k] == hx[i][k] + 1), ry[k] == hy[i][k])) for i in range(num_hor) for k in range(lim + 1) ]
prevent_coll_h_h = [ Not(And(hx[i][k] == hx[j][k] - 1, hy[i][k] == hy[j][k])) for i in range(num_hor) for j in range(num_hor) for k in range(lim+1)]
prevent_coll_v_v = [ Not(And(vy[i][k] == vy[j][k] - 1, vx[i][k] == vx[j][k])) for i in range(num_ver) for j in range(num_ver) for k in range(lim+1)]

avoidance = prevent_mine_h + prevent_mine_v + prevent_mine_r + prevent_coll_h_v + prevent_coll_r_h + prevent_coll_r_v + prevent_coll_v_v + prevent_coll_h_h

move_v = [[And(Or(vy[i][j+1] == vy[i][j] + 1, vy[i][j+1] == vy[i][j] - 1), vx[i][j+1] == vx[i][j]) for j in range(lim) ] for i in range(num_ver)]
move_h = [[And(Or(hx[i][j+1] == hx[i][j] + 1 , hx[i][j+1] == hx[i][j] - 1 ), hy[i][j+1] == hy[i][j])  for j in range(lim) ] for i in range(num_hor)]
move_r = [And(Or(rx[j+1] == rx[j] + 1 , rx[j+1] == rx[j] - 1 ), ry[j+1] == ry[j]) for j in range(lim)]

move_control_v = [Or(move_v[i][j], And(vy[i][j+1] == vy[i][j], vx[i][j+1] == vx[i][j])) for i in range(num_ver) for j in range(lim)]
move_control_h = [Or(move_h[i][j], And(hx[i][j+1] == hx[i][j], hy[i][j+1] == hy[i][j])) for i in range(num_hor) for j in range(lim)]
move_control_r = [Or(move_r[j], And(rx[j+1] == rx[j], ry[j+1] == ry[j])) for j in range(lim)]

move_control = move_control_v + move_control_h + move_control_r

only_one_move_vr = [Not( And( move_v[i][j] , move_r[j]) ) for i in range(num_ver) for j in range(lim) ]
only_one_move_hr = [Not(And( move_h[i][j] , move_r[j]) ) for i in range(num_hor) for j in range(lim)]
only_one_move_vh = [Not(And( move_h[hi][j] , move_v[vi][j]) ) for vi in range(num_ver) for hi in range(num_hor) for j in range(lim)]
only_one_move_hh = [Not( And( move_h[i][k] , move_h[j][k] , i!=j ) ) for i in range(num_hor) for j in range(num_hor) for k in range(lim)]
only_one_move_vv = [Not( And( move_v[i][k] , move_v[j][k] , i!=j ) ) for i in range(num_ver) for j in range(num_ver) for k in range(lim)]

or_move_v = [ Or([move_v[vi][j] for vi in range(num_ver)]) for j in range(lim)]
or_move_h = [ Or([move_h[hi][j] for hi in range(num_hor)]) for j in range(lim)]

atleast_one_move = [Or( or_move_h[j] , or_move_v[j] , move_r[j] ) for j in range(lim) ]

only_one_move = only_one_move_hh + only_one_move_hr + only_one_move_vh + only_one_move_vv + only_one_move_vr

s = Solver()
s.add(limits + init + goal + avoidance + only_one_move + atleast_one_move + move_control)
if s.check() == sat:
    m = s.model()
    for j in range(lim):
        if m.evaluate(move_r[j]):
            if(m[rx[j+1]] == simplify(m[rx[j]] + 1)):
                print(m[ry[j]], m[rx[j+1]] ,sep=',')
                if(simplify(m[rx[j]] + 1) == n-2):
                    break
            else:
                print(m[ry[j]],m[rx[j]],sep=',')
        for i in range(num_ver):
            if m.evaluate(move_v[i][j]):
                if(m[vy[i][j+1]] == simplify(m[vy[i][j]] + 1)):
                    print(m[vy[i][j+1]], m[vx[i][j]],sep=',')
                else:
                    print(m[vy[i][j]], m[vx[i][j]],sep=',')

        for i in range(num_hor):
            if m.evaluate(move_h[i][j]):
                if(m[hx[i][j+1]] == simplify(m[hx[i][j]] + 1)):
                    print(m[hy[i][j]], m[hx[i][j+1]] ,sep=',')
                else:
                    print(m[hy[i][j]], m[hx[i][j]],sep=',')
else:
    print("unsat")
