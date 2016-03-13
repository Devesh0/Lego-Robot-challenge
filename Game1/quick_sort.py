def quickSort(arr):
    lower = []
    pivotList = []
    greater = []
    if len(arr) <= 1:
        return arr
    else:
        pivot = arr[0]
        for i in arr:
            if i < pivot:
                lower.append(i)
                print("Lower elements" +str(lower))
            elif i > pivot:
                greater.append(i)
                print("Greater elements " +str(greater))
            else:
                pivotList.append(i)
                print("Pivot Elements"+ str(pivotList))
        lower = quickSort(lower)
        greater = quickSort(greater)
        return lower + pivotList + greater
 
aList = ['fuel','battery','door','cpu','antenna','propulsion']    
aList = quickSort(aList)
print(aList)
