"""Demo-only drawing of commanded legs, using a replaceable geometry adapter."""
import pygame

import eth_theme as eth
from leg_geometry import Q8FiveBarGeometry
from pose_preview import DESCRIPTIONS, PREVIEW_SIZE
from body_pose import support_transform


class KinematicPreview:
    # Servo pair order: FL, FR, BL, BR. Body spacing/perspective are illustrative.
    ANCHORS = ((53, 27), (53, -27), (-53, 27), (-53, -27))
    MOUNT_HEIGHT = 43.36

    def __init__(self, geometry=None):
        self.geometry = geometry if geometry is not None else Q8FiveBarGeometry()
        self.surface = pygame.Surface((PREVIEW_SIZE[0] * 2, PREVIEW_SIZE[1] * 2))
        self.fonts = {size: pygame.font.Font(None, size * 2) for size in (17, 18, 22, 23, 44)}
        self.body_mode = None
        self.body_transform = None
        self.routine_label = None

    @staticmethod
    def project(x, side, height):
        return 400 + x * 2.6 + side * 1.05, 340 + side * .5 - height * 2.75

    def project_leg(self, index, geometry):
        x, side = self.ANCHORS[index]
        return tuple(self.body_project(x + px, side, self.MOUNT_HEIGHT - py)
                     for px, py in geometry.points)

    def body_project(self, x, side, height):
        if self.body_transform is not None:
            return self.project(*self.body_transform.apply(x, side, height - self.MOUNT_HEIGHT))
        return self.project(x, side, height)

    @staticmethod
    def _point(point):
        return tuple(round(value * 2) for value in point)

    def _line(self, color, a, b, width=2):
        pygame.draw.line(self.surface, color, self._point(a), self._point(b), width * 2)

    def _circle(self, color, point, radius):
        pygame.draw.circle(self.surface, color, self._point(point), radius * 2)

    def _polygon(self, color, points):
        pygame.draw.polygon(self.surface, color, [self._point(p) for p in points])

    def _text(self, value, size, color, xy):
        self.surface.blit(self.fonts[size].render(value, True, color), self._point(xy))

    def render(self, positions, name):
        """Render exactly one eight-angle sample; no gait or pose generation."""
        if len(positions) != 8:
            raise ValueError('Expected eight commanded joint angles')
        legs = [self.geometry.solve(*positions[i:i + 2]) for i in range(0, 8, 2)]
        self.body_transform = (support_transform(legs, self.ANCHORS, self.MOUNT_HEIGHT)
                               if self.body_mode else None)
        self.surface.fill(eth.PANEL)
        accent = eth.GAIT_COLORS[name]
        title, description = DESCRIPTIONS[name]
        if self.routine_label:
            title = self.routine_label.title()
            description = ('Commanded greeting sequence.' if self.routine_label == 'GREETING'
                           else 'Firmware joint commands; no flight simulation.')
        self._text('KINEMATIC DEMO', 18, accent, (28, 23))
        self._text(title, 44, eth.TEXT_PRIMARY, (28, 47))
        self._text(description, 23, eth.TEXT_SECONDARY, (28, 95))
        for x in range(-100, 101, 25):
            self._line(eth.GRID, self.project(x, -58, 0), self.project(x, 58, 0), 1)
        for side in range(-50, 51, 25):
            self._line(eth.GRID, self.project(-105, side, 0), self.project(105, side, 0), 1)

        def draw_leg(index, far=False):
            leg = legs[index]
            points = self.project_leg(index, leg)
            rod = eth.mix(accent, eth.PANEL, .45) if far else accent
            shank = eth.mix(eth.SHANK, eth.PANEL, .38) if far else eth.SHANK
            for a, b in leg.links:
                driven = a in leg.mounts or b in leg.mounts
                self._line(eth.OUTLINE, points[a], points[b], 11 if driven else 9)
                self._line(rod if driven else shank, points[a], points[b], 7 if driven else 5)
            for index_point, point in enumerate(points):
                mount = index_point in leg.mounts
                self._circle(eth.CHASSIS_FILL if mount else eth.OUTLINE, point, 9 if mount else 5)
                self._circle(eth.JOINT_CORE if mount else rod, point, 4 if mount else 3)
            self._circle(eth.OUTLINE, points[leg.foot], 7)
            self._circle(rod, points[leg.foot], 4)
            if not far:
                self._text('FRONT' if index == 0 else 'REAR', 17, eth.TEXT_MUTED,
                           (points[leg.foot][0] - 23, 387))

        draw_leg(3, True)
        draw_leg(1, True)
        corners = [self.body_project(x, side, self.MOUNT_HEIGHT)
                   for x, side in ((-69, -24), (69, -24), (69, 24), (-69, 24))]
        self._polygon(eth.CHASSIS_SHADOW, [(x, y + 13) for x, y in corners])
        self._polygon(eth.CHASSIS_FILL, corners)
        pygame.draw.lines(self.surface, eth.CHASSIS_EDGE, True,
                          [self._point(p) for p in corners], 4)
        for side in (-11, 10):
            self._line(eth.BATTERY, self.body_project(-36, side, self.MOUNT_HEIGHT + 3),
                       self.body_project(36, side, self.MOUNT_HEIGHT + 3), 10)
        draw_leg(2)
        draw_leg(0)
        self._line(accent, (686, 176), (725, 176))
        self._polygon(accent, [(725, 176), (716, 171), (716, 181)])
        self._text('FRONT', 17, accent, (681, 151))
        self._line(eth.PANEL_RULE, (28, 420), (772, 420), 1)
        detail = 'Kinematic contacts / no physics' if self.body_mode else 'Commanded angles / fixed chassis'
        self._text(detail, 22, eth.TEXT_SECONDARY, (28, 437))
        self._text('Kinematic animation', 18, eth.TEXT_MUTED, (615, 439))
        return pygame.transform.smoothscale(self.surface, PREVIEW_SIZE)
