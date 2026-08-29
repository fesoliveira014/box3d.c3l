module b3;

macro bool WorldId.is_null(self) => self.index1 == 0;
macro bool BodyId.is_null(self) => self.index1 == 0;
macro bool ShapeId.is_null(self) => self.index1 == 0;
macro bool JointId.is_null(self) => self.index1 == 0;
macro bool ContactId.is_null(self) => self.index1 == 0;
