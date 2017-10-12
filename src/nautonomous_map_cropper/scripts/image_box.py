from position import Position

class ImageBox():
    def __init__(self, top_left, top_right, bottom_left, bottom_right, position):
        self.top_left_ = top_left
        self.top_right_ = top_right
        self.bottom_left_ = bottom_left
        self.bottom_right_ = bottom_right
        self.position_ = position

    def top_left(self):
        return self.top_left_

    def top_right(self):
        return self.top_right_

    def bottom_left(self):
        return self.bottom_left_

    def bottom_right(self):
        return self.bottom_right_
    
    def width(self):
        return max(self.bottom_right().x(), self.top_right().x()) - min(self.top_left().x(), self.bottom_left().x())

    def height(self):
        if self.position_ == Position.CENTER:
            return max(self.top_left().y(), self.top_right().y()) - min(self.bottom_left().y(), self.bottom_right().y())
        elif self.position_ == Position.TOPLEFT:
            return min(self.bottom_left().y(), self.bottom_right().y()) - max(self.top_left().y(), self.top_right().y())

    def __repr__(self):
        return "\n | tl " + str(self.top_left_) + "\n | tr " + str(self.top_right_) + "\n | bl " + str(self.bottom_left_) + "\n | br " + str(self.bottom_right_)