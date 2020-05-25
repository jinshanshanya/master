import os


def func1():
    #with open('insMsgLog58.txt') as f1, open('swap_a.txt', 'w') as f2:
    with open('insMsgLog.txt') as f1, open('swap_a.txt', 'w') as f2:
        data = f1.read()
        datas = data.split('\n')
        count = 0
        for content in datas:
            if 'Lat' in content or 'Lon' in content or 'Heading:' in content:
                tep = content.split(':')
                count += 1
                if count % 3 == 0:
                    f2.write(tep[1] + '\n')
                else:
                    f2.write(tep[1] + ',')

    # os.remove('insMsgLog.txt')
    os.rename('swap_a.txt', 'point.txt')


def func2():
    with open('point.txt') as f, open('waypoint.csv.swap', 'w') as p:
        line = f.read()
        line = line.replace(' ', '')
        p.write(line)
        print(line)

    os.remove('point.txt')
    os.rename('waypoint.csv.swap', 'waypoint2.csv')


func1()
func2()
