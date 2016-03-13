def swap(i, j):                    
    alist[i], alist[j] = alist[j], alist[i] 

def heap(end,i):   
    l=2 * i + 1  
    r=2 * (i + 1)   
    max=i   
    if l < end and alist[i] < alist[l]:   
        max = l
    if r < end and alist[max] < alist[r]:   
        max = r
    if max != i:   
        swap(i, max)   
        heap(end, max)

def heap_sort():     
    end = len(alist)   
    start = end // 2 - 1 
    for i in range(start, -1, -1):   
        heap(end, i)
    for i in range(end-1, 0, -1):   
        swap(i, 0)   
        heap(i, 0)
        

alist = ['fuel','propulsion','batteries','antenna','door','cpu']
heap_sort()
print(alist)







