"""Illustrative support-contact placement, not dynamics or stability prediction."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BodyTransform:
    pitch: float = 0.0
    x: float = 0.0
    height: float = 43.36

    def apply(self, x, side, height):
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        return self.x + x*c - height*s, side, self.height + x*s + height*c


def support_transform(legs, anchors, mount_height):
    """Place the rear pair at their ground anchors; front contact can slide.

    Sitting levels front/rear feet, which requires pitch as well as translation.
    With folded front legs, the same nose-up inclination leaves them free for
    rearing/greeting. Pitch is a geometric convention: two rear contacts alone
    cannot determine body orientation. No balance or inferred force is involved.

    Symmetric folded entry has no support stance. Acquire/release rear contact
    smoothly over the first 5 mm of front/rear foot-depth difference, preserving
    the existing fixed-chassis walking view at symmetric return configurations.
    """
    feet = [(anchors[i][0] + leg.points[leg.foot][0], leg.points[leg.foot][1])
            for i, leg in enumerate(legs)]
    front = tuple((feet[0][j] + feet[1][j]) / 2 for j in (0, 1))
    rear = tuple((feet[2][j] + feet[3][j]) / 2 for j in (0, 1))
    difference = front[1] - rear[1]
    contact = min(1.0, abs(difference) / 5)
    contact = contact * contact * (3 - 2 * contact)
    pitch = math.atan2(abs(difference), max(1.0, front[0] - rear[0])) * contact
    c, s = math.cos(pitch), math.sin(pitch)
    rear_anchor = (anchors[2][0] + anchors[3][0]) / 2
    x = rear_anchor - (rear[0]*c + rear[1]*s)
    height = max(0.0, rear[1]*c - rear[0]*s)
    return BodyTransform(pitch, x * contact, mount_height + (height - mount_height) * contact)
