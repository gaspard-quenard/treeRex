from graphviz import Digraph
import argparse
import hashlib
import sys


def extract_parenthesized_tokens(s):
    """Return top-level '(...)' tokens from a line.

    Tokens may contain spaces; we parse by balanced parentheses.
    We also support an optional '%<id>' suffix immediately following the
    closing ')', e.g. '(move a b)%42'.
    """
    tokens = []
    depth = 0
    start = None
    for i, ch in enumerate(s):
        if ch == '(':
            if depth == 0:
                start = i
            depth += 1
        elif ch == ')':
            if depth == 0:
                continue
            depth -= 1
            if depth == 0 and start is not None:
                end = i + 1
                # Optional '%id' suffix right after ')'
                if end < len(s) and s[end] == '%':
                    j = end + 1
                    while j < len(s) and not s[j].isspace():
                        j += 1
                    end = j
                tokens.append(s[start:end])
                start = None
    return tokens



if __name__ == '__main__':

    all_primitive_tree_nodes = []

    # Graphviz treats ':' in edge endpoints as the port separator (node:port).
    # Our action strings contain ':' (e.g., 'Q_0,0_location:0_...'), which makes
    # graphviz-python split the identifier and generate invalid DOT.
    # Fix: use a DOT-safe internal node id and keep the original string as label.
    node_id_by_label: dict[str, str] = {}

    def node_id(label: str) -> str:
        existing = node_id_by_label.get(label)
        if existing is not None:
            return existing
        digest = hashlib.md5(label.encode('utf-8')).hexdigest()[:12]
        nid = f"n_{digest}"
        node_id_by_label[label] = nid
        return nid

    parser = argparse.ArgumentParser(description='Render a DAG from an all_paths_*.txt file.')
    parser.add_argument('filename', help='Input file (e.g., all_paths_2.txt)')
    parser.add_argument(
        '--prim',
        action='store_true',
        help='Only show nodes that are in the primitive tree (isInPrimitiveTree==1), and edges between them.',
    )
    args = parser.parse_args()

    filename = args.filename

    # First read the file

    # Then create the graph
    dot = Digraph(name='DAG', graph_attr={'label': 'Orthogonal edges', 'splines': 'ortho', 'nodesep': '0.8'}, node_attr={'shape': 'box'})

    # Then add the nodes
    with open(filename, 'r') as f:
        connection = False
        for line in f:
            if (not connection):
                if (line == '\n'):
                    connection = True
                else:
                    # Remove the \n and spaces at the end
                    line = line.rstrip()
                    if not line:
                        continue

                    toks = line.split()
                    if not toks:
                        raise ValueError(f"Could not parse node line (missing '(...)'): {line!r}")

                    name = toks[0]

                    # Extract the remainder after the first token.
                    stripped = line.lstrip()
                    if stripped.startswith(name):
                        rest = stripped[len(name):].strip()
                    else:
                        rest = line[line.find(name) + len(name):].strip()
                    rest_parts = rest.split()
                    if len(rest_parts) < 2:
                        raise ValueError(
                            "Could not parse node flags (expected: '<node> isAction isInPrimitiveTree'): "
                            + repr(line)
                        )
                    isAction = int(rest_parts[0])
                    isInPrimitiveTree = int(rest_parts[1])

                    if args.prim and isInPrimitiveTree == 0:
                        continue

                    nid = node_id(name)
                    if (isAction == 1 and isInPrimitiveTree == 1):
                        dot.node(nid, name, fillcolor='green', style='filled')
                        all_primitive_tree_nodes.append(nid)
                    elif (isAction == 1):
                        dot.node(nid, name, fillcolor='yellow', style='filled')
                    else:
                        dot.node(nid, name) #, fillcolor='red', style='filled')
            else:
                # Remove the \n and spaces at the end
                line = line.rstrip()
                if not line:
                    continue

                # toks = extract_parenthesized_tokens(line)
                toks = line.split()
                if not toks:
                    continue
                source = toks[0]
                source_id = node_id_by_label.get(source)
                if source_id is None:
                    continue
                for target in toks[1:]:
                    target_id = node_id_by_label.get(target)
                    if target_id is None:
                        continue
                    if (source_id in all_primitive_tree_nodes and target_id in all_primitive_tree_nodes):
                        dot.edge(source_id, target_id, color='red')
                    else:
                        dot.edge(source_id, target_id)

    print(dot.source)
    # Name of render of filename without extension _DAG.pdf
    name = filename.split(".")[0] + "_DAG"    
    dot.render(view=True, filename=name)
