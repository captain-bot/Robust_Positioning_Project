def solution(S):
    phone_number, call_duration = list(), list()
    data = S.split()
    for d in data:
        newd = d.split(",")
        call_time = newd[0].split(":")
        call_time = int(call_time[0])*3600 + int(call_time[1])*60 + int(call_time[2])
        call_duration.append(call_time)
        ph_num = newd[1].split("-")
        ph_num = ph_num[0] + ph_num[1] + ph_num[2]
        phone_number.append(int(ph_num))

    call_duration.sort()
    call_duration.pop()
    print(call_duration)

    total_cost = 0
    for i in range(len(phone_number)-1):
        time_in_sec = call_duration[i]
        if time_in_sec < 300:
            total_cost += 3*time_in_sec
        elif time_in_sec == 300:
            total_cost += 5*150
        elif time_in_sec > 300:
            if time_in_sec % 60 == 0:
                ttime = time_in_sec/60
            else:
                ttime = time_in_sec//60 + 1
            total_cost += ttime*150
    return total_cost


S = '00:01:07,400-234-090\n00:05:01,701-080-080\n00:05:00,400-234-090'
cost = solution(S)
print(cost)