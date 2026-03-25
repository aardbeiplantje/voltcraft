#!/usr/bin/perl
#
# voltcraft.pl - BLE controller for Voltcraft SEM-6000 wall socket meter/switch
#
# Protocol reference:
#   https://github.com/Heckie75/voltcraft-sem-6000/blob/master/sem-6000.exp
#
# GATT profile (SEM-6000):
#   Service UUID:  fff0
#   Command char:  fff3  (write CRC-framed commands here, ATT Write Command)
#   Notify char:   fff4  (subscribe CCCD; responses arrive as ATT notifications)
#
# Packet framing (requests and responses):
#   [0x0F][LEN][payload...][CHK][0xFF][0xFF]
#   LEN = len(payload) + 1
#   CHK = (1 + sum(payload)) & 0xFF
#
# Commands (payload bytes, framing added automatically):
#   Auth:        0x17 0x00 0x00 <p0 p1 p2 p3> 0x00 0x00 0x00 0x00
#   Measurement: 0x04 0x00 0x00 0x00
#   Switch:      0x03 0x00 <0x01|0x00> 0x00 0x00
#
# Usage:
#   ./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin 0000] --status
#   ./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin 0000] --meter
#   ./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin 0000] --on
#   ./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin 0000] --off
#   ./voltcraft.pl -d AA:BB:CC:DD:EE:FF [--pin 0000] --toggle
#

use strict;
use warnings;
use bytes;
use Getopt::Long;

my %opts = (
    device            => undef,
    pin               => '0000',
    addr_type         => 'public',
    connect_timeout   => 5,
    debug             => 0,
    service_uuid      => 'fff0',
    command_char_uuid => 'fff3',
    notify_char_uuid  => 'fff4',
    wait_notify_ms    => 3000,
);

my ($do_status, $do_meter, $do_on, $do_off, $do_toggle, $do_name) = (0) x 6;
my $set_name;
my $set_pin;

GetOptions(
    'device|d=s'          => \$opts{device},
    'pin|p=s'             => \$opts{pin},
    'addr-type=s'         => \$opts{addr_type},
    'connect-timeout=f'   => \$opts{connect_timeout},
    'service-uuid=s'      => \$opts{service_uuid},
    'command-char-uuid=s' => \$opts{command_char_uuid},
    'notify-char-uuid=s'  => \$opts{notify_char_uuid},
    'wait-notify-ms=i'    => \$opts{wait_notify_ms},
    'status'              => \$do_status,
    'meter'               => \$do_meter,
    'on'                  => \$do_on,
    'off'                 => \$do_off,
    'toggle'              => \$do_toggle,
    'name'               => \$do_name,
    'set-name=s'         => \$set_name,
    'set-pin=s'          => \$set_pin,
    'debug|v+'            => \$opts{debug},
    'help|h'              => sub { print_usage(); exit 0; },
) or do { print_usage(); exit 1; };

unless ($opts{device}) {
    print STDERR "Error: -d / --device is required\n";
    print_usage();
    exit 1;
}
unless ($opts{device} =~ /^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$/) {
    print STDERR "Error: Invalid Bluetooth address format\n";
    exit 1;
}
unless ($opts{pin} =~ /^\d{4}$/) {
    print STDERR "Error: --pin must be exactly 4 digits (default: 0000)\n";
    exit 1;
}
if ($opts{addr_type} !~ /^(?:public|random)$/i) {
    print STDERR "Error: --addr-type must be 'public' or 'random'\n";
    exit 1;
}

for my $k (qw(service_uuid command_char_uuid notify_char_uuid)) {
    (my $u = lc $opts{$k}) =~ s/^0x//;
    unless ($u =~ /^[0-9a-f]{4}$/) {
        print STDERR "Error: --$k must be a 4-hex-digit UUID16 (e.g. fff0)\n";
        exit 1;
    }
    $opts{$k} = $u;
}

# Default: show both if nothing requested
if (!$do_status && !$do_meter && !$do_on && !$do_off && !$do_toggle && !$do_name
    && !defined($set_name) && !defined($set_pin)) {
    $do_status = $do_meter = 1;
}

if (defined $set_name) {
    if (length($set_name) < 1 || length($set_name) > 18) {
        print STDERR "Error: --set-name must be 1..18 characters\n";
        exit 1;
    }
    if ($set_name =~ /[^\x20-\x7E]/) {
        print STDERR "Error: --set-name only supports printable ASCII (space through ~)\n";
        exit 1;
    }
}

if (defined $set_pin && $set_pin !~ /^\d{4}$/) {
    print STDERR "Error: --set-pin must be exactly 4 digits\n";
    exit 1;
}

my $sem = Voltcraft::SEM->new(%opts);
exit $sem->run(
    status => $do_status,
    meter  => $do_meter,
    on     => $do_on,
    off    => $do_off,
    toggle => $do_toggle,
    name   => $do_name,
    set_name => $set_name,
    set_pin  => $set_pin,
);

# ============================================================================
# Voltcraft::SEM class
# ============================================================================

package Voltcraft::SEM;

use Errno   qw(EAGAIN EINPROGRESS);
use Fcntl   qw(O_NONBLOCK F_SETFL F_GETFL);
use Socket  qw(SOCK_SEQPACKET SOL_SOCKET SO_ERROR);

use constant {
    AF_BLUETOOTH              => 31,
    BTPROTO_L2CAP             => 0,
    BDADDR_LE_PUBLIC          => 0x01,
    BDADDR_LE_RANDOM          => 0x02,

    ATT_FIND_BY_TYPE_REQ      => 0x06,
    ATT_FIND_BY_TYPE_RSP      => 0x07,
    ATT_READ_BY_TYPE_REQ      => 0x08,
    ATT_READ_BY_TYPE_RSP      => 0x09,
    ATT_WRITE_REQ             => 0x12,
    ATT_WRITE_RSP             => 0x13,
    ATT_WRITE_CMD             => 0x52,
    ATT_HANDLE_VALUE_NOTIF    => 0x1B,
    ATT_EXCHANGE_MTU_REQ      => 0x02,
    ATT_EXCHANGE_MTU_RSP      => 0x03,
    ATT_ERROR_RSP             => 0x01,

    GATT_PRIMARY_SERVICE_UUID => 0x2800,
    GATT_CHARACTERISTIC_UUID  => 0x2803,

    # SEM-6000 command payload IDs
    SEM_CMD_GET_NAME          => 0x01,
    SEM_CMD_AUTH              => 0x17,
    SEM_CMD_SWITCH            => 0x03,
    SEM_CMD_MEASURE           => 0x04,
};

sub new {
    my ($class, %o) = @_;
    bless {
        device          => $o{device},
        pin             => $o{pin} // '0000',
        addr_type       => lc($o{addr_type} // 'public'),
        connect_timeout => $o{connect_timeout} // 5,
        debug           => $o{debug} // 0,

        service_uuid    => hex($o{service_uuid}),
        command_uuid    => hex($o{command_char_uuid}),
        notify_uuid     => hex($o{notify_char_uuid}),
        wait_notify_ms  => $o{wait_notify_ms} // 3000,

        socket          => undef,
        handle_command  => 0,
        handle_cccd     => 0,
    }, $class;
}

# ============================================================================
# Top-level workflow
# ============================================================================

sub run {
    my ($self, %todo) = @_;

    print "Voltcraft SEM BLE Tool\n";
    print "======================\n\n";
    print "Device: $self->{device}\n";

    unless ($self->ble_connect()) {
        print STDERR "ERROR: BLE connection failed\n";
        return 1;
    }
    unless ($self->discover_handles()) {
        print STDERR "ERROR: fff0 service / fff3 command handle not found\n";
        $self->ble_disconnect();
        return 1;
    }

    # Request larger MTU so multi-byte notifications fit in one packet.
    $self->exchange_mtu(160);

    $self->subscribe_notify();

    unless ($self->auth()) {
        print STDERR "ERROR: Authentication failed (check --pin, default is 0000)\n";
        $self->ble_disconnect();
        return 1;
    }

    my $rc = 0;

    if (defined $todo{set_name}) {
        $self->set_name($todo{set_name}) or $rc = 1;
    }

    if (defined $todo{set_pin}) {
        $self->set_pin($todo{set_pin}) or $rc = 1;
    }

    if ($todo{name}) {
        my $name = $self->get_name();
        if (defined $name && length($name)) {
            print "Name:         $name\n";
        } elsif (defined $name) {
            print "Name:         (not set)\n";
        } else {
            print STDERR "ERROR: Could not read device name\n";
            $rc = 1;
        }
    }

    # Toggle: must measure first to learn current state, then switch.
    if ($todo{toggle}) {
        my $m = $self->measure();
        if (!defined $m) {
            print STDERR "ERROR: Toggle failed — could not read current state\n";
            $rc = 1;
        } else {
            $self->switch_socket($m->{power} ? 0 : 1) or $rc = 1;
        }
    }

    $self->switch_socket(1) or $rc = 1 if $todo{on};
    $self->switch_socket(0) or $rc = 1 if $todo{off};

    if ($todo{status} || $todo{meter}) {
        my $m = $self->measure();
        if (!defined $m) {
            print STDERR "ERROR: Measurement failed\n";
            $rc = 1;
        } else {
            print "Switch:       " . ($m->{power} ? 'ON' : 'OFF') . "\n" if $todo{status};
            if ($todo{meter}) {
                printf "Voltage:      %d V\n",    $m->{voltage};
                printf "Current:      %.3f A\n",  $m->{ampere};
                printf "Power:        %.3f W\n",  $m->{watt};
                printf "Frequency:    %d Hz\n",   $m->{frequency};
                printf "Power factor: %.3f\n",    $m->{power_factor};
                printf "Total energy: %.3f kWh\n", $m->{total}
                    if $m->{total} >= 0;
            }
        }
    }

    $self->ble_disconnect();
    return $rc;
}

# ============================================================================
# BLE connection / GATT discovery
# ============================================================================

sub ble_connect {
    my ($self) = @_;
    my @oct    = split(':', $self->{device});
    my $bdaddr = pack('C6', map { hex($_) } reverse @oct);
    my $atype  = ($self->{addr_type} eq 'random') ? BDADDR_LE_RANDOM : BDADDR_LE_PUBLIC;

    socket(my $sock, AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP) or return 0;

    # Bind local LE-public any-address, ATT fixed channel (CID 4)
    bind($sock, pack('S S a6 S S', AF_BLUETOOTH, 0, "\0" x 6, 4, BDADDR_LE_PUBLIC))
        or do { close($sock); return 0; };

    my $peer = pack('S S a6 S S', AF_BLUETOOTH, 0, $bdaddr, 4, $atype);
    fcntl($sock, F_SETFL, fcntl($sock, F_GETFL, 0) | O_NONBLOCK);

    my $connected = connect($sock, $peer);
    if (!$connected && !($! == EINPROGRESS || $! == EAGAIN)) {
        $self->debug("connect() immediate fail: $!");
        close($sock); return 0;
    }

    unless ($connected) {
        my $deadline = time() + ($self->{connect_timeout} > 0 ? $self->{connect_timeout} : 5);
        my $done     = 0;
        while (time() < $deadline) {
            my $fh   = fileno($sock);
            my $wvec = '';
            vec($wvec, $fh, 1) = 1;
            my $n = select(undef, $wvec, my $evec = $wvec, 0.5);
            next unless defined($n) && $n > 0;
            my $err = getsockopt($sock, SOL_SOCKET, SO_ERROR);
            my $eno = $err ? unpack('I', $err) : 0;
            if (!$eno)                                { $done = 1; last; }
            next if $eno == EINPROGRESS || $eno == EAGAIN;
            last;
        }
        unless ($done) { $self->debug('connect timeout'); close($sock); return 0; }
    }

    my $err = getsockopt($sock, SOL_SOCKET, SO_ERROR);
    if ($err && unpack('I', $err)) { close($sock); return 0; }

    fcntl($sock, F_SETFL, fcntl($sock, F_GETFL, 0) & ~O_NONBLOCK);
    $self->{socket} = $sock;
    return 1;
}

sub discover_handles {
    my ($self) = @_;

    my $rsp = $self->att_request(
        pack('C S< S< S< S<',
            ATT_FIND_BY_TYPE_REQ, 0x0001, 0xFFFF,
            GATT_PRIMARY_SERVICE_UUID, $self->{service_uuid}),
        3.0
    );
    return 0 unless defined $rsp && length($rsp) >= 5
                    && ord(substr($rsp, 0, 1)) == ATT_FIND_BY_TYPE_RSP;

    my ($svc_start, $svc_end) = unpack('S< S<', substr($rsp, 1, 4));
    $self->debug(sprintf('Service 0x%04X: 0x%04X-0x%04X',
        $self->{service_uuid}, $svc_start, $svc_end));

    my $start = $svc_start;
    while ($start <= $svc_end) {
        my $crsp = $self->att_request(
            pack('C S< S< S<', ATT_READ_BY_TYPE_REQ, $start, $svc_end, GATT_CHARACTERISTIC_UUID),
            2.0
        );
        last unless defined $crsp && length($crsp) >= 2;
        my $cop = ord(substr($crsp, 0, 1));
        last if $cop != ATT_READ_BY_TYPE_RSP;

        my $elen = ord(substr($crsp, 1, 1));
        last if $elen < 7;

        my ($pos, $last_decl) = (2, $start);
        while ($pos + $elen <= length($crsp)) {
            my $e    = substr($crsp, $pos, $elen);
            my $decl = unpack('S<', substr($e, 0, 2));
            my $val  = unpack('S<', substr($e, 3, 2));
            my $uuid = unpack('S<', substr($e, 5, 2));
            $self->debug(sprintf('  UUID 0x%04X -> handle 0x%04X', $uuid, $val));

            $self->{handle_command} = $val     if $uuid == $self->{command_uuid};
            $self->{handle_cccd}    = $val + 1 if $uuid == $self->{notify_uuid};

            $last_decl = $decl;
            $pos += $elen;
        }
        $start = $last_decl + 1;
    }

    $self->debug(sprintf('command=0x%04X  cccd=0x%04X',
        $self->{handle_command}, $self->{handle_cccd}));
    return $self->{handle_command} ? 1 : 0;
}

sub ble_disconnect {
    my ($self) = @_;
    if ($self->{socket}) {
        close($self->{socket});
        $self->{socket} = undef;
    }
}

# ============================================================================
# ATT helpers
# ============================================================================

# Generic write-then-read for ATT request/response pairs (GATT discovery, CCCD).
sub att_request {
    my ($self, $req, $timeout) = @_;
    $timeout //= 2.0;
    return unless $self->{socket};
    syswrite($self->{socket}, $req) or return;

    my $rin = '';
    vec($rin, fileno($self->{socket}), 1) = 1;
    my $n = select(my $rout = $rin, undef, undef, $timeout);
    return unless defined($n) && $n > 0;

    my ($rsp, $r) = ('');
    $r = sysread($self->{socket}, $rsp, 512);
    return unless defined($r) && $r > 0;
    return $rsp;
}

# Request MTU 160 so the device can send large notifications in one BLE packet.
sub exchange_mtu {
    my ($self, $mtu) = @_;
    $mtu //= 160;
    my $rsp = $self->att_request(pack('C S<', ATT_EXCHANGE_MTU_REQ, $mtu), 2.0);
    return unless defined $rsp && length($rsp) >= 3
                  && ord(substr($rsp, 0, 1)) == ATT_EXCHANGE_MTU_RSP;
    $self->debug(sprintf('MTU: client=%d server=%d', $mtu, unpack('S<', substr($rsp, 1, 2))));
}

# Enable notifications on the fff4 CCCD.
sub subscribe_notify {
    my ($self) = @_;
    return 1 unless $self->{handle_cccd};
    my $req = pack('C S< S<', ATT_WRITE_REQ, $self->{handle_cccd}, 0x0001);
    my $rsp = $self->att_request($req, 2.0);
    my $ok  = defined($rsp) && length($rsp) >= 1
              && ord(substr($rsp, 0, 1)) == ATT_WRITE_RSP;
    $self->debug($ok ? 'Notifications enabled' : 'CCCD write failed (continuing)');
    return $ok;
}

# ============================================================================
# SEM-6000 protocol: framing, command/response, auth, measure, switch
# ============================================================================

# Build framed SEM packet: [0x0F][LEN][payload...][CHK][0xFF][0xFF]
# LEN = len(payload)+1, CHK = (1 + sum(payload)) & 0xFF
sub _build_packet {
    my (@payload) = @_;
    my $len = scalar(@payload) + 1;
    my $chk = 1;
    $chk = ($chk + $_) & 0xFF for @payload;
    return pack('C*', 0x0F, $len, @payload, $chk, 0xFF, 0xFF);
}

# Send a framed command via ATT Write Command (0x52, no ATT-layer ACK),
# then accumulate ATT Handle Value Notification(s) until a complete response
# frame is assembled. Returns arrayref of decoded payload bytes, or undef.
sub command_req {
    my ($self, @payload) = @_;
    return unless $self->{socket} && $self->{handle_command};

    my $pkt = _build_packet(@payload);
    $self->debug('TX: ' . join(' ', map { sprintf '%02X', $_ } unpack('C*', $pkt)));

    syswrite($self->{socket}, pack('C S<', ATT_WRITE_CMD, $self->{handle_command}) . $pkt)
        or return;

    # Accumulate notification bytes until the SEM response frame is complete.
    # The device may split long responses across multiple ATT notification PDUs.
    my @buf;
    my $timeout = $self->{wait_notify_ms} / 1000.0;

    while (1) {
        my $rin = '';
        vec($rin, fileno($self->{socket}), 1) = 1;
        my $ready = select(my $rout = $rin, undef, undef, $timeout);
        return unless defined($ready) && $ready > 0;

        my ($raw, $r) = ('');
        $r = sysread($self->{socket}, $raw, 512);
        return unless defined($r) && $r > 0;

        my $op = ord(substr($raw, 0, 1));
        unless ($op == ATT_HANDLE_VALUE_NOTIF) {
            $self->debug(sprintf('Unexpected ATT opcode 0x%02X, skipping', $op));
            $timeout = 0.3;
            next;
        }
        # [op(1)][handle(2)][value...] — append value bytes to accumulation buffer
        push @buf, unpack('C*', substr($raw, 3));
        $self->debug('RX chunk: ' . join(' ', map { sprintf '%02X', $_ } unpack('C*', substr($raw, 3))));

        next unless @buf >= 2 && $buf[0] == 0x0F;
        # Frame complete when: 2 header bytes + (LEN-1) payload + 1 CHK = LEN+2 bytes total
        last if scalar(@buf) >= $buf[1] + 2;
        $timeout = 1.0;
    }

    return unless @buf >= 2 && $buf[0] == 0x0F;
    my $expected = $buf[1];
    return unless scalar(@buf) >= $expected + 2;

    my $got_chk = $buf[$expected + 1];
    my $chk = 1;
    $chk = ($chk + $buf[$_]) & 0xFF for 2..$expected;
    $self->debug(sprintf('CHK %02X vs %02X%s', $chk, $got_chk, $chk == $got_chk ? ' OK' : ' MISMATCH'));

    my @rsp = @buf[2..$expected];
    $self->debug('RSP: ' . join(' ', map { sprintf '%02X', $_ } @rsp));
    return \@rsp;
}

# Authenticate with PIN (default 0000).
# Must succeed before any other command.
# Payload: [0x17 0x00 0x00 p0 p1 p2 p3 0x00 0x00 0x00 0x00]
# PIN digits are sent as integers 0-9 (not ASCII).
sub auth {
    my ($self) = @_;
    my @pin = map { int($_) } split(//, $self->{pin});
    my $rsp  = $self->command_req(SEM_CMD_AUTH, 0x00, 0x00, @pin, 0x00, 0x00, 0x00, 0x00);
    my $ok   = defined($rsp) && @$rsp >= 3
               && $rsp->[0] == SEM_CMD_AUTH
               && $rsp->[1] == 0
               && $rsp->[2] == 0;
    $self->debug($ok ? 'Auth OK' : 'Auth FAILED: '
        . (defined $rsp ? join(' ', map { sprintf '%02X', $_ } @$rsp) : 'no response'));
    return $ok;
}

# Request a measurement snapshot.
# Payload: [0x04 0x00 0x00 0x00]
# Response layout (indices into the returned payload array):
#   [0]=0x04 [1]=0x00 [2]=power_on [3-5]=watt*1000 (24-bit BE)
#   [6]=voltage  [7-8]=ampere*1000 (16-bit BE)  [9]=frequency
#   [12-15]=total_kWh*1000 (32-bit BE) -- only in 16-byte response (hw<3)
sub measure {
    my ($self) = @_;
    my $rsp = $self->command_req(SEM_CMD_MEASURE, 0x00, 0x00, 0x00);

    unless (defined $rsp
            && (@$rsp == 16 || @$rsp == 14)
            && $rsp->[0] == SEM_CMD_MEASURE
            && $rsp->[1] == 0) {
        $self->debug('measure failed: '
            . (defined $rsp ? join(' ', map { sprintf '%02X', $_ } @$rsp) : 'no response'));
        return undef;
    }

    my $power   =  $rsp->[2] == 1 ? 1 : 0;
    my $watt    = (($rsp->[3] << 16) | ($rsp->[4] << 8) | $rsp->[5]) / 1000.0;
    my $voltage =   $rsp->[6];
    my $ampere  = (($rsp->[7] <<  8) | $rsp->[8]) / 1000.0;
    my $freq    =   $rsp->[9];

    my $pf;
    if ($power && $voltage > 0 && $ampere > 0) {
        $pf = $watt / ($voltage * $ampere);
        $pf = 1.0 if $pf > 1.0;
    } else {
        $pf = $power ? 1.0 : 0.0;
    }

    my $total = -1;
    if (@$rsp == 16) {
        $total = (($rsp->[12] << 24) | ($rsp->[13] << 16)
                | ($rsp->[14] <<  8) |  $rsp->[15]) / 1000.0;
    }

    return {
        power        => $power,
        watt         => $watt,
        voltage      => $voltage,
        ampere       => $ampere,
        frequency    => $freq,
        power_factor => $pf,
        total        => $total,
    };
}

# Fetch the device's stored name/description.
# Strategy (each tried in order until a non-empty name is found):
#  1. SEM-6000 custom settings command (0x01): user-defined name from companion app.
#     Older firmware (hw<3) has extra fields before the name, so scan for the first
#     contiguous run of printable ASCII bytes rather than assuming a fixed offset.
#  2. BLE Generic Access Device Name (UUID 0x2A00).
#  3. BlueZ filesystem cache (~/.var/lib/bluetooth or /var/lib/bluetooth).
sub get_name {
    my ($self) = @_;

    my $rsp = $self->command_req(SEM_CMD_GET_NAME, 0x00, 0x00, 0x00);
    if (defined $rsp && @$rsp >= 3 && $rsp->[0] == SEM_CMD_GET_NAME) {
        $self->debug('GET_NAME rsp: ' . join(' ', map { sprintf '%02X', $_ } @$rsp));
        # Scan from offset 2 for the first run of printable ASCII chars.
        # Non-name fields (relay state, time, flags) are typically low control bytes.
        my $name = '';
        for my $i (2..$#$rsp) {
            my $b = $rsp->[$i];
            if ($b >= 0x20 && $b <= 0x7E) {
                $name .= chr($b);
            } elsif (length $name) {
                last;  # end of printable run
            }
        }
        return $name if length($name);
        $self->debug('SEM custom name is empty, trying BLE Device Name');
    } else {
        $self->debug('SEM get-name command failed: '
            . (defined $rsp ? join(' ', map { sprintf '%02X', $_ } @$rsp) : 'no response'));
    }

    # Fall back: read Generic Access Device Name characteristic (UUID 0x2A00).
    my $ble_name = $self->read_ble_device_name();
    return $ble_name if defined $ble_name && length $ble_name;

    # Last resort: BlueZ device cache on disk (populated from prior BLE scans).
    return $self->read_bluez_cached_name();
}

# Read the BLE Generic Access Device Name (UUID 0x2A00) using ATT_READ_BY_TYPE_REQ.
# Loops to discard any stale ATT_HANDLE_VALUE_NOTIF packets left in the socket.
# Response layout: [op(1)][item_len(1)][handle(2)][value...]
sub read_ble_device_name {
    my ($self) = @_;
    return undef unless $self->{socket};
    syswrite($self->{socket},
        pack('C S< S< S<', ATT_READ_BY_TYPE_REQ, 0x0001, 0xFFFF, 0x2A00))
        or return undef;

    my $deadline = time() + 2.0;
    while (1) {
        my $remaining = $deadline - time();
        last if $remaining <= 0;
        my $rin = '';
        vec($rin, fileno($self->{socket}), 1) = 1;
        my $n = select(my $rout = $rin, undef, undef, $remaining);
        last unless defined($n) && $n > 0;
        my ($raw, $r) = ('');
        $r = sysread($self->{socket}, $raw, 512);
        last unless defined($r) && $r > 0;
        my $op = ord(substr($raw, 0, 1));
        if ($op == ATT_READ_BY_TYPE_RSP) {
            return '' if length($raw) < 5;
            # value starts after [opcode(1), item_len(1), handle(2)]
            my $name = substr($raw, 4);
            $name =~ s/\x00.*//s;
            $self->debug("BLE Device Name: '$name'");
            return $name;
        }
        $self->debug(sprintf('read_ble_device_name: skipping opcode 0x%02X', $op));
    }
    return undef;
}

# Read the device name from the BlueZ on-disk cache.
# BlueZ writes discovered device info to /var/lib/bluetooth/<adapter>/<device>/info.
# Works when the device has previously been scanned or connected via BlueZ.
sub read_bluez_cached_name {
    my ($self) = @_;
    my $dev = uc($self->{device});
    for my $dir (glob('/var/lib/bluetooth/*/')) {
        my $info = "${dir}${dev}/info";
        open(my $fh, '<', $info) or next;
        while (<$fh>) {
            if (/^Name=(.+)/) {
                chomp(my $name = $1);
                close $fh;
                $self->debug("BlueZ cached name: '$name'");
                return $name;
            }
        }
        close $fh;
    }
    return undef;
}

# Set the device custom name.
# Payload: [0x02 0x00 <name bytes padded to 18> 0x00 0x00]
# Success response: [0x02 0x00 0x00]
sub set_name {
    my ($self, $name) = @_;
    return 0 unless defined $name;

    my @name_bytes = map { ord($_) } split(//, $name);
    if (@name_bytes > 18) {
        print STDERR "ERROR: Name too long (max 18 chars)\n";
        return 0;
    }
    push @name_bytes, (0) x (18 - scalar(@name_bytes));

    my $rsp = $self->command_req(0x02, 0x00, @name_bytes, 0x00, 0x00);
    my $ok  = defined($rsp) && @$rsp >= 3
              && $rsp->[0] == 0x02
              && $rsp->[1] == 0x00
              && $rsp->[2] == 0x00;
    if ($ok) {
        print "Renamed:      $name\n";
    } else {
        print STDERR "ERROR: Rename command failed\n";
    }
    return $ok;
}

# Change device PIN.
# Payload: [0x17 0x00 0x01 <new pin digits> <current pin digits>]
# Success response starts with [0x17 0x00 0x00 0x01].
sub set_pin {
    my ($self, $new_pin) = @_;
    return 0 unless defined $new_pin;

    unless ($new_pin =~ /^\d{4}$/) {
        print STDERR "ERROR: New PIN must be exactly 4 digits\n";
        return 0;
    }

    my @new = map { int($_) } split(//, $new_pin);
    my @old = map { int($_) } split(//, $self->{pin});

    my $rsp = $self->command_req(SEM_CMD_AUTH, 0x00, 0x01, @new, @old);
    my $ok  = defined($rsp) && @$rsp >= 4
              && $rsp->[0] == SEM_CMD_AUTH
              && $rsp->[1] == 0x00
              && $rsp->[2] == 0x00
              && $rsp->[3] == 0x01;
    if ($ok) {
        $self->{pin} = $new_pin;
        print "PIN set:      $new_pin\n";
    } else {
        print STDERR "ERROR: PIN change failed (check current --pin)\n";
    }
    return $ok;
}

# Send switch on/off command.
# Payload: [0x03 0x00 <1|0> 0x00 0x00]
# Reference check: rsp[0]==0x03 && rsp[2]==0 (rsp[1] is intentionally not checked).
sub switch_socket {
    my ($self, $on) = @_;
    my $rsp = $self->command_req(SEM_CMD_SWITCH, 0x00, ($on ? 1 : 0), 0x00, 0x00);
    my $ok  = defined($rsp) && @$rsp >= 3
              && $rsp->[0] == SEM_CMD_SWITCH
              && $rsp->[2] == 0;
    $ok ? printf("Switched: %s\n", $on ? 'ON' : 'OFF')
        : print(STDERR "ERROR: Switch command failed\n");
    return $ok;
}

sub debug {
    my ($self, $msg) = @_;
    print "  [DEBUG] $msg\n" if $self->{debug};
}

1;

# ============================================================================
# Main package helpers
# ============================================================================

package main;

sub print_usage {
    print <<"EOF";
Usage: $0 -d AA:BB:CC:DD:EE:FF [--pin NNNN] [actions] [options]

Actions (one or more; defaults to --status --meter):
  --name          Show the device's stored name/description
    --set-name TXT  Rename device custom name (1..18 printable ASCII chars)
    --set-pin NNNN  Change device PIN to new 4-digit PIN
  --status        Show switch state (ON/OFF)
  --meter         Show voltage, current, power, frequency, power factor
  --on            Turn socket on
  --off           Turn socket off
  --toggle        Toggle socket state

Required:
  -d, --device ADDR       BLE MAC address

Options:
  --pin NNNN              4-digit PIN (default: 0000)
  --addr-type TYPE        public|random (default: public)
  --connect-timeout SEC   Connect timeout in seconds (default: 5)
  --wait-notify-ms N      Max ms to wait per notification chunk (default: 3000)
  -v, --debug             Verbose output (repeat for hex dumps)
  -h, --help              Show this help

GATT UUID overrides (4-hex-digit):
  --service-uuid UUID       Service UUID (default: fff0)
  --command-char-uuid UUID  Command characteristic (default: fff3)
  --notify-char-uuid UUID   Notify characteristic (default: fff4)

Examples:
  $0 -d B3:00:00:00:73:C0 --status
  $0 -d B3:00:00:00:73:C0 --meter
  $0 -d B3:00:00:00:73:C0 --pin 1234 --on
  $0 -d B3:00:00:00:73:C0 --toggle
  $0 -d B3:00:00:00:73:C0 --status --meter -v
EOF
}

