
import matplotlib.pyplot as plt
import numpy as np
import yaml


def show_traject(config):
    with open("para.txt", 'r') as file:
        pra = []
        lines = []
        for i in range(3):
            line = float(file.readline())

            lines.append(line)
        pra.append(lines)

        lines = []
        for i in range(2):
            line = float(file.readline())

            lines.append(line)
        pra.append(lines)

        lines = []
        for i in range(2):
            line = float(file.readline())
  # 确保读取的行不为空
            lines.append(line)
        pra.append(lines)


    max_sim_size = config["simulator"]["MAX_SIM_SIZE"]
    t_step = config["simulator"]["STEP_SIZE"]

    t = np.arange(0, max_sim_size, t_step)

    #t = np.linspace(0, 20, 20)
    x = pra[2][0] * t + pra[2][1]
    y = pra[1][0] * t + pra[1][1]  # y = 7x
    z = pra[0][0] + pra[0][1] * t + pra[0][2] * t ** 2

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制散点图
    ax.scatter(x, y, z, color='r', label='Scatter Points')
    ax.plot(x, y, z, color='b')

    # 设置轴标签
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    # 设置标题
    #ax.set_title('3D Parabolic Curve Fitting without Noise')

    # 设置 x 和 y 轴的刻度范围一致
    ax.set_xlim([0, max_sim_size])
    ax.set_ylim([0, max_sim_size])
    ax.set_zlim([0, max_sim_size])

    # 显示图例
    ax.legend()

    # 显示图形
    plt.show()
    print()

def show(config):
    max_sim_size = config["simulator"]["MAX_SIM_SIZE"]

    data = np.loadtxt("output_trajection.txt", dtype=str)
    x = data[:, 2].astype(float)  # 第三列
    y = data[:, 3].astype(float)  # 第四列
    z = data[:, 4].astype(float)  # 第五列

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制3D散点图
    ax.scatter(x, y, z, c='r', marker='o')

    # 设置轴标签
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    ax.set_xlim([0, max_sim_size])
    ax.set_ylim([0, max_sim_size])
    ax.set_zlim([0, max_sim_size])
    # 显示图形
    plt.show()




class Hitter:
    def __init__(self):
        self.name = "hitter"

        self.current_x = []
        self.current_y = []
        self.current_z = []

class Obj:
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []

def show_hitter(canvas, tx, ty, tz, config):
    max_sim_size = config["simulator"]["MAX_SIM_SIZE"]
    #fig = plt.figure()
    #canvas = fig.add_subplot(111, projection='3d')

    # test--
    # canvas.scatter(hitter.estimated_obj_next_pos[0],
    #               hitter.estimated_obj_next_pos[1],
    #               hitter.estimated_obj_next_pos[2],s=30)

    #canvas.scatter(ox, oy, oz, color='r', label='Scatter Points')
    #plt.cla()
    #fig.clear()

    #canvas = fig.add_subplot(111, projection='3d')
    #canvas.scatter(ox, oy, oz, color='r')
    #canvas.scatter(hitter.estimated_obj_next_pos[0],hitter.estimated_obj_next_pos[1],hitter.estimated_obj_next_pos[2], s=3, color="black")
    #canvas.plot(ox, oy, oz, color='g')

    canvas.plot(tx, ty, tz, marker="s", lw=0.5, ms=0.4, c="Blue")

    canvas.set_xlabel('X axis')
    canvas.set_ylabel('Y axis')
    canvas.set_zlabel('Z axis')

    # 设置标题
    #canvas.set_title('3D Parabolic Curve Fitting without Noise')

    # 设置 x 和 y 轴的刻度范围一致
    if config["simulator"]["COORDINATE"] == "cartesian":
        canvas.set_xlim([0, max_sim_size])
        canvas.set_ylim([0, max_sim_size])
        canvas.set_zlim([0, max_sim_size])

    elif config["simulator"]["COORDINATE"] == "GPS":
        canvas.set_xlim([config["min_loc"][0], config["max_loc"][0]])
        canvas.set_ylim([config["min_loc"][1], config["max_loc"][1]])
        canvas.set_zlim([config["min_loc"][2], config["max_loc"][2]])
    # 显示图例
    #canvas.legend()
    plt.draw()

def show_obj(canvas, ox, oy, oz, config):
    max_sim_size = config["simulator"]["MAX_SIM_SIZE"]
    #fig = plt.figure()
    #canvas = fig.add_subplot(111, projection='3d')

    # test--
    # canvas.scatter(hitter.estimated_obj_next_pos[0],
    #               hitter.estimated_obj_next_pos[1],
    #               hitter.estimated_obj_next_pos[2],s=30)

    #canvas.scatter(ox, oy, oz, color='r', label='Scatter Points')
    #plt.cla()
    #fig.clear()

    #canvas = fig.add_subplot(111, projection='3d')
    canvas.scatter(ox, oy, oz, color='r')
    #canvas.scatter(hitter.estimated_obj_next_pos[0],hitter.estimated_obj_next_pos[1],hitter.estimated_obj_next_pos[2], s=3, color="black")
    canvas.plot(ox, oy, oz, color='g')

    #canvas.plot(tx, ty, tz, marker="s", lw=0.5, ms=0.4, c="Blue")

    canvas.set_xlabel('X axis')
    canvas.set_ylabel('Y axis')
    canvas.set_zlabel('Z axis')

    # 设置标题
    #canvas.set_title('3D Parabolic Curve Fitting without Noise')

    # 设置 x 和 y 轴的刻度范围一致
    if config["simulator"]["COORDINATE"] == "cartesian":
        canvas.set_xlim([0, max_sim_size])
        canvas.set_ylim([0, max_sim_size])
        canvas.set_zlim([0, max_sim_size])

    elif config["simulator"]["COORDINATE"] == "GPS":
        canvas.set_xlim([config["min_loc"][0], config["max_loc"][0]])
        canvas.set_ylim([config["min_loc"][1], config["max_loc"][1]])
        canvas.set_zlim([config["min_loc"][2], config["max_loc"][2]])

    # 显示图例
    #canvas.legend()
    plt.draw()




if __name__ == "__main__":

    #result = subprocess.run([r"C:\Users\Z\Desktop\work\蛋\cpp\missle\x64\Release\missle.exe"],
    #                         capture_output=True,
    #                         text=True)
    # print("Standard Output:", result.stdout)
    # print("Standard Error:", result.stderr)


    # -----------------修改此处-------------------------------------------------------
    yamlpath = r"config.yaml"
    trajection_path = r"output_trajection.txt"
    #--------------------------------------------------------------------------------



    #os.system(r"C:\Users\Z\Desktop\work\蛋\cpp\missle\missle\x64\Release\missle.exe")

    fig = plt.figure()
    canvas = fig.add_subplot(111, projection='3d')
    with open(yamlpath, "r") as file:
        config = yaml.safe_load(file)



    NUM_INTERCEPTOR = config["interceptor"]["NUM_INTERCEPTOR"]
    max_sim_size = config["simulator"]["MAX_SIM_SIZE"]
    loc = config["simulator"]["INITIAL_LOC"]
    max_loc = [0, 0, 0]
    for i in range(2):
        max_loc[i] = max_sim_size * 0.02 + loc[i]
    max_loc[2] = max_sim_size * 1000 + loc[2]

    config["max_loc"] = max_loc
    config["min_loc"] = loc

    obj = Obj()

    hitters = []
    for i in range(NUM_INTERCEPTOR):
        hitter = Hitter()
        hitters.append(hitter)

    with open(trajection_path, 'r') as file:
        lines = file.readlines()

    turn = len(lines)/(NUM_INTERCEPTOR+1)



    for i in range(int(turn)):
        plt.cla()
        for j in range(NUM_INTERCEPTOR):
            text = lines[i*(NUM_INTERCEPTOR+1) + j].split()
            hitters[j].current_x.append(float(text[2]))
            hitters[j].current_y.append(float(text[3]))
            hitters[j].current_z.append(float(text[4]))
            show_hitter(canvas, hitters[j].current_x, hitters[j].current_y, hitters[j].current_z, config)


        text = lines[i * (NUM_INTERCEPTOR+1) + NUM_INTERCEPTOR].split()
        obj.x.append(float(text[2]))
        obj.y.append(float(text[3]))
        obj.z.append(float(text[4]))
        show_obj(canvas, obj.x, obj.y, obj.z, config)
        plt.pause(0.01)

    plt.show()
    #show_traject(config)
    #show(config)
