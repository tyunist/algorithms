
# Wheel random sampling: 
# given point list p2 with N elements and respective weight list w, draw N sample with replacement and possibility 
# proportion to w. 
index = random.randint(0,N)
beta = 0
maxw = max(w)
for i in range(N):
    beta = beta + random.uniform(0, maxw)
    while beta > w[index]:
        beta = beta - w[index]
        index = (index + 1)%N
    p3.append(p2[index])
p = p3

print p #please leave this print statement here for grading!
