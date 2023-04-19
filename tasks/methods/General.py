from Navigation import Navigation
from Perception import Perception


class General:
    def __init__(self):
        self.navigation = Navigation()
        self.perception = Perception()

    @staticmethod
    def find_available_seat(self, *guests, num_seats: int = 5):
        # Cria uma lista de assentos vazios
        seats = [False] * num_seats
        # Calcula os pontos médios de cada assento
        seat_points = [640 * (2 * i + 1) / (2 * num_seats) for i in range(num_seats)]

        # Itera sobre cada convidado
        for guest in guests:
            # Verifica em qual intervalo de assentos o convidado se encaixa
            for i in range(num_seats - 1):
                if seat_points[i] <= guest < seat_points[i + 1]:
                    # Marca o assento como ocupado
                    seats[i + 1] = True
                    # Para de procurar por assentos
                    break
            else:
                # Se o convidado não se encaixar em nenhum intervalo, assume o primeiro assento como ocupado
                seats[0] = True

        # Itera sobre cada assento e retorna o primeiro que estiver vazio
        for i, seat in enumerate(seats):
            if not seat:
                return seat_points[i]

        # Retorna None se todos os assentos estiverem ocupados
        return None

    def lugar_vazio(self, guest1, guest2, guest3):
        num_lugares = 5
        seat1 = False
        seat2 = True
        seat3 = False
        seat4 = False
        seat5 = True

        seat1_point = 640 / (2 * num_lugares)
        seat2_point = 640 * 3 / (2 * num_lugares)
        seat3_point = 640 * 5 / (2 * num_lugares)
        seat4_point = 640 * 7 / (2 * num_lugares)

        if 0 <= guest1 < (1 / num_lugares * 640) or 0 <= guest2 < (1 / num_lugares * 640) or 0 <= guest3 < (
                1 / num_lugares * 640):
            seat1 = True

        if (2 / num_lugares * 640) < guest1 < (3 / num_lugares * 640) or (2 / num_lugares * 640) < guest2 < (
                3 / num_lugares * 640) or (2 / num_lugares * 640) < guest3 < (3 / num_lugares * 640):
            seat3 = True

        if (3 / num_lugares * 640) < guest1 < (4 / num_lugares * 640) or (3 / num_lugares * 640) < guest2 < (
                4 / num_lugares * 640) or (3 / num_lugares * 640) < guest3 < (4 / num_lugares * 640):
            seat4 = True

        print(seat1)
        print("1", guest1)
        print(guest2)
        print(guest3)

        if seat1 == False:
            return (seat1_point)
        elif seat1 == True and seat3 == False:
            return (seat3_point)
        elif seat1 == True and seat3 == True and seat4 == False:
            return (seat4_point)
        else:
            return None

    def center_filter(self, vel, time):
        check_x = False
        check_y = False
        check_pose = False
        while not check_pose:
            if not check_y:
                resp = self.color_filter('')
                if resp.center_y < 246.5:  # 158.5:
                    self.navigation.move('foward', vel, time)
                    self.navigation.move('stop', 0, 0)
                elif resp.center_y > 266.6:  # 178.5:
                    self.navigation.move('backward', vel, time)
                    self.navigation.move('stop', 0, 0)
                else:
                    check_y = True
            if not check_x:
                resp = self.color_filter('')
                if resp.center_x < 312.5:  # 291.0:
                    self.navigation.move('right', vel, time)
                    self.navigation.move('stop', 0, 0)
                elif resp.center_x > 332.5:  # 311.0:
                    self.navigation.move('left', vel, time)
                    self.navigation.move('stop', 0, 0)
                else:
                    check_x = True
                    check_y = False
            else:
                check_pose = True
        return True
