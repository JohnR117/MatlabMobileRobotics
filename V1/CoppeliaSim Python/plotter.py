import matplotlib.pyplot as plt

class RobotDataPlotter:
    def __init__(self):
        self.fig_robot_pos = plt.figure()
        self.ax_robot_pos = self.fig_robot_pos.add_subplot(111)
        self.ax_robot_pos.set_xlabel('Coordenada X')
        self.ax_robot_pos.set_ylabel('Coordenada Y')
        self.ax_robot_pos.set_title('Posición del Robot')
        self.robot_pos_line, = self.ax_robot_pos.plot([], [], color='red', label='Posición del robot')

        self.fig_robot_ortt = plt.figure()
        self.ax_robot_ortt = self.fig_robot_ortt.add_subplot(111)
        self.ax_robot_ortt.set_xlabel('Eje')
        self.ax_robot_ortt.set_ylabel('Ángulo (radianes)')
        self.ax_robot_ortt.set_title('Orientación del Robot')
        self.robot_ortt_bar = self.ax_robot_ortt.bar(['Ángulo Z'], [0], color='blue')

        self.fig_pos_err = plt.figure()
        self.ax_pos_err = self.fig_pos_err.add_subplot(111)
        self.ax_pos_err.set_xlabel('Tipo de error')
        self.ax_pos_err.set_ylabel('Valor')
        self.ax_pos_err.set_title('Error de Posición')
        self.pos_err_bar = self.ax_pos_err.bar(['Error de posición'], [0], color='green')

        self.fig_orr_err = plt.figure()
        self.ax_orr_err = self.fig_orr_err.add_subplot(111)
        self.ax_orr_err.set_xlabel('Tipo de error')
        self.ax_orr_err.set_ylabel('Valor')
        self.ax_orr_err.set_title('Error de Orientación')
        self.orr_err_bar = self.ax_orr_err.bar(['Error de orientación'], [0], color='purple')

        self.fig_hed_err = plt.figure()
        self.ax_hed_err = self.fig_hed_err.add_subplot(111)
        self.ax_hed_err.set_xlabel('Tipo de error')
        self.ax_hed_err.set_ylabel('Valor')
        self.ax_hed_err.set_title('Error de Dirección')
        self.hed_err_bar = self.ax_hed_err.bar(['Error de dirección'], [0], color='orange')

        plt.show(block=False)

    def update_robot_data(self, rob_poss, rob_ortt, pos_err, orr_err, hed_err):
        self.robot_pos_line.set_data(rob_poss[0], rob_poss[1])
        self.robot_ortt_bar[0].set_height(rob_ortt[2])
        self.pos_err_bar[0].set_height(pos_err)
        self.orr_err_bar[0].set_height(orr_err)
        self.hed_err_bar[0].set_height(hed_err)

        self.fig_robot_pos.canvas.draw()
        self.fig_robot_ortt.canvas.draw()
        self.fig_pos_err.canvas.draw()
        self.fig_orr_err.canvas.draw()
        self.fig_hed_err.canvas.draw()