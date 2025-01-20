# A Brief Introduction to Behavior Trees

ROS 2 provides a powerful tool for controlling the movement and behavior of robots: the ***Behavior Tree***.
Behavior are simple XML files that you can create which can then be used to execute complex behaviors for the robot.
Look at this simple, hypothetical example of a behavior tree:

```xml
<RepeatUntilSuccessful>
  <Fallback>
    <MoraleHasImproved/>
    <GiveABeating/>
  </Fallback>
</RepeatUntilSuccessful>
```

This behavior tree will keep trying to improve the morale of the robot by beating it until morale has improved.

You can also visualize the behavior tree in a graphical way, like this:

<figure style="text-align: center;" markdown="1">
  <a name="fig:beatings_will_continue"></a>
  <img
  src="../media/beatings_will_continue.png"
  alt="The beatings will continue until morale improves">
  <figcaption>The beatings will continue until morale improves"</figcaption>
</figure>

To make the the discussion a bit easier to read, I will shorten the term ***behavior tree*** to ***BT*** from now on.

## What is a Behavior Tree?

A BT is composed of `nodes`. Graphically, think of `nodes` as the boxes in the [picture](#fig:beatings_will_continue) above.
`Nodes` are connected to each other in a `parent` to `child` relationship, represented by the lines connecting the boxes in the picture.
Every `node` has exactly one `parent` and may have none, one, or multiple `children`.
Only one node in the tree has no parent, and that node is called the `root` node.

There are four general categories of nodes that can be used in a behavior tree:

* `Action` These are nodes that usually cause things to happen.
   Examples of action nodes might be: `MoveForward`, `FollowPath`, or `GoToDock`.
* `Condition` Represents a condition that must be met for the robot to continue executing some part of the tree.
  Examples of condition nodes might be: `IsBatteryLow`, `IsDistanceTravelledGreaterThan`, or `IsRobotAtDock`.
* `Control` nodes control the flow of execution.
  Examples of control nodes might be:
  * `Fallback` Represents a sequence of behaviors that are tried in order until one succeeds.
  * `RecoveryNode` Will try to recover from failure of the first child `node` by executing the second child `node`.
  * `Sequence` Performs the behavior of the child `nodes` in order.
* `Decorator` Modifies the behavior of the child `node`.
  Examples of decorator nodes might be:
  * `RateController` Limits the frequency at which the child `node` is executed.
  * `SingleTrigger` Ensures that the child `node` is only executed once.
  * `Inverter` Inverts the result of the child `node`.

There are a set of general nodes provided by the BT library, and additional, navigation specific nodes
provided by the `nav2` package, but you can also create your own nodes.

## How Does a Behavior Tree Work?

A `node` represents some kind of behavior. The behavior takes place when the node receives a signal called `tick`.
When a node receives a `tick`, it begins performing its behavior and returns one of three states: `SUCCESS`, `FAILURE`, or `RUNNING`.
Well, some nodes are not allowed to return all of those states, but we will get to that later.

