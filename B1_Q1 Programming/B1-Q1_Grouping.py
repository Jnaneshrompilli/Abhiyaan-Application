"""
*********************************************************************************************************************
                     SECTION-B
SECTION B.1
1) An integer array, “Arr”, having n elements, consists of only 0’s
and 1’s. In one move, you can choose two adjacent indices and swap
their values. Return the minimum number of moves required so that all
1’s of “Arr” are grouped together

Constraint: Only values on adjacent indices can be swapped

Answer explanation:

Take an array [0,0,1,0,1,0,1,0]
Array Index  - 0 1 2 3 4 5 6 7
1] Calculate the median of indices whoose stored value is equal to 1. In this case indices are 2,4,6
   And the median is 4
2] Fix the array's value (i.e value 1) at index 4
    i) Move 1's on left side of index 4 closer to 1(at index 4) by swapping at a specific condition and count each swap
    ii) Apply same for the right side of index 4   
    iii) Result - [0,0,0,1,1,1,0,0]      

Note: If we have two median values, check the number of swaps required for both values and choose the minimum

********************************************************************************************************************"""


def swaparray(arr_input, index, median, m):
    arr_temp = arr_input[:m]

    # For counting no of swaps occured at left and right of the median
    count_left = 0
    count_right = 0
    median = index[int(median - 1)]

    # Iterating over left side of index 4
    no_of_iterations_l = median - 1
    for swap_l in range(0, no_of_iterations_l):
        # print(f"Swap no : {swap_l}")
        for i in range(median - 1, 0, -1):
            if (
                arr_temp[i] == 0 and arr_temp[i - 1] == 1
            ):  # Condition for swapping 1 and 0
                temp = arr_temp[i]
                arr_temp[i] = arr_temp[i - 1]  # Swapping Values
                arr_temp[i - 1] = temp

                count_left = count_left + 1

            else:
                pass

    # Iterating over right side of index 4
    no_of_iterations_r = n - median - 2
    for swap_r in range(0, no_of_iterations_r):
        # print(f"Swap No: {swap_r}")
        for j in range(median + 1, n - 1):
            if (
                arr_temp[j] == 0 and arr_temp[j + 1] == 1
            ):  # Condition for swapping 1 and 0
                temp = arr_temp[j]
                arr_temp[j] = arr_temp[j + 1]  # Swapping Values
                arr_temp[j + 1] = temp

                count_right = count_right + 1
            else:
                pass

    # Sum of swap counts on left and right sides
    swap_count = count_right + count_left

    return swap_count, arr_temp


# Function to group all 1's together
def findMinSwaps(arr_input, n):

    # Storing the indices of value 1 in arrray index
    index = []
    for i in range(0, n):
        if arr_input[i] == 1:
            index.append(i)

    # Finding Position of median
    count = len(index)
    if (count) % 2 != 0:
        median_pos = (count + 1) / 2
    # Case where two medians exist
    else:
        median1_pos = count / 2
        median2_pos = (count / 2) + 1

    # Calling function swaparray to group 1's together by passing arguments Input array,Index array of 1's, Median position on Index array, Size of Input array

    # If single median exists
    try:
        s_count, s_arr = swaparray(arr_input, index, median_pos, n)

        # Returning grouped array and no of minimum swaps required
        return s_arr, s_count

    # If two medians exists
    except:
        s_count1, s_arr1 = swaparray(arr_input, index, median1_pos, n)
        s_count2, s_arr2 = swaparray(arr_input, index, median2_pos, n)
        # Returning the minimum swap count
        if s_count1 <= s_count2:
            return s_arr1, s_count1
        else:
            return s_arr2, s_count2


# -------------Interpret from here-----------
n = int(input("Enter the size of the array: "))
arr = []

# Storing the user input in an array
for k in range(n):
    value = int(input(f"Enter number(0/1) {k+1}: "))
    arr.append(value)

print(f"\n-----Before Swapping-----\nThe Array is: {arr}")

# Calling function
try:
    swapped_arr, swap_count = findMinSwaps(arr, n)
    print("\n----After Swapping----")
    print(f"The Array is: {swapped_arr}")
    print(f"No of Swaps: {swap_count}\n")

# Case where when user enters only zeroes
except:
    print("\n----After Swapping----")
    print(f"The Array is: {arr}")
    print(f"No of Swaps: 0\n")
