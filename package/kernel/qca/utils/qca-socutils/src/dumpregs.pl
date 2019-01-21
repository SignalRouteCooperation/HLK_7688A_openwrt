#!/usr/bin/microperl

#
# Copyright (c) 2013 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

# Command line configuration
# %Config{dump} = contains the list of tables to dump [ gpio, stereo, mbox ]
my %h_Config;

# h_registers contains the list of all the registers we may want to dump
# This structure is organized per register range in the SoC
# Each sub-hash should contain at least two keys:
#  - base: must contain the base address of these registers
#  - .*: must contain the name offset (vs base) to access that register
my %h_registers = (
	# GPIO registers definition
	"gpio" => {
		readcb				=> \&DumpRegs,
		base				=> 0x18040000,
		regsdef => {
			"GPIO_OE"			=> 0x0,
			"GPIO_IN"			=> 0x4,
			"GPIO_OUT"			=> 0x8,
			"GPIO_SET"			=> 0xC,
			"GPIO_CLEAR"			=> 0x10,
			"GPIO_INT"			=> 0x14,
			"GPIO_INT_TYPE"			=> 0x18,
			"GPIO_INT_POLARITY"		=> 0x1C,
			"GPIO_INT_PENDING"		=> 0x20,
			"GPIO_INT_MASK"			=> 0x24,
			"GPIO_IN_ETH_SWITCH_LED"	=> 0x28,
			"GPIO_OUT_FUNCTION0"		=> 0x2C,
			"GPIO_OUT_FUNCTION1"		=> 0x30,
			"GPIO_OUT_FUNCTION2"		=> 0x34,
			"GPIO_OUT_FUNCTION3"		=> 0x38,
			"GPIO_OUT_FUNCTION4"		=> 0x3C,
			"GPIO_OUT_FUNCTION5"		=> 0x40,
			"GPIO_IN_ENABLE0"		=> 0x44,
			"GPIO_IN_ENABLE1"		=> 0x48,
			"GPIO_IN_ENABLE2"		=> 0x4C,
			"GPIO_IN_ENABLE3"		=> 0x50,
			"GPIO_IN_ENABLE4"		=> 0x54,
			"GPIO_IN_ENABLE9"		=> 0x68,
			"GPIO_FUNCTION"			=> 0x6C,
		},
	},
	"gmac" => {
		readcb				=> \&DumpRegs,
		base				=> 0x18070000,
		regsdef => {
			"GMAC_ETH_CFG"			=> 0x00,
			"GMAC_LUTS_AGER_INTR"		=> 0x04,
			"GMAC_LUTS_AGER_INTR_MASK"	=> 0x08,
			"GMAC_GE0_RX_DATA_CRC_CNTRL"	=> 0x0c,
			"GMAC_GE0_RX_DATA_CRC"		=> 0x10,
			"GMAC_SGMII_RESET"		=> 0x14,
			"GMAC_SGMII_SERDES"		=> 0x18,
			"GMAC_MR_AN_CONTROL"		=> 0x1c,
			"GMAC_MR_AN_STATUS"		=> 0x20,
			"GMAC_AN_ADV_ABILITY"		=> 0x24,
			"GMAC_AN_LINK_PARTNER_ABILITY"	=> 0x28,
			"GMAC_AN_NP_TX"			=> 0x2c,
			"GMAC_AN_LP_NP_RX"		=> 0x30,
			"GMAC_SGMII_CONFIG"		=> 0x34,
			"GMAC_SGMII_MAC_RX_CONFIG"	=> 0x38,
			"GMAC_SGMII_MAC_TX_CONFIG"	=> 0x3c,
			"GMAC_SGMII_MDIO_TX"		=> 0x40,
			"GMAC_SGMII_MDIO_RX"		=> 0x44,
			"GMAC_EEE"			=> 0x48,
			"GMAC_SGMII_RESOLVE"		=> 0x54,
			"GMAC_SGMII_INTERRUPT"		=> 0x5c,
			"GMAC_SGMII_INTERUP_MASK"	=> 0x60,
			"GMAC_PBRS_STATUS"		=> 0x64,
		},
	},
	"gmac0" => {
		readcb				=> \&DumpRegs,
		base				=> 0x19000000,
		regsdef => {
			"GMAC0_MAC_CFG_1"		=> 0x00,
			"GMAC0_MAC_CFG_2"		=> 0x04,
			"GMAC0_IPG_IFG"			=> 0x08,
			"GMAC0_HALF_DUPLEX"		=> 0x0c,
			"GMAC0_MAX_FRM_LEN"		=> 0x10,
			"GMAC0_MII_CONFIG"		=> 0x20,
			"GMAC0_MII_COMMAND"		=> 0x24,
			"GMAC0_MII_ADDRESS"		=> 0x28,
			"GMAC0_MII_CONTROL"		=> 0x2c,
			"GMAC0_MII_STATUS"		=> 0x30,
			"GMAC0_MII_INDICATOR"		=> 0x34,
			"GMAC0_IFACE_CONTROL"		=> 0x38,
			"GMAC0_IFACE_STATUS"		=> 0x3c,
			"GMAC0_STA_ADDRESS1"		=> 0x40,
			"GMAC0_STA_ADDRESS2"		=> 0x44,
			"GMAC0_ETH_CFG0"		=> 0x48,
			"GMAC0_ETH_CFG1"		=> 0x4c,
			"GMAC0_ETH_CFG2"		=> 0x50,
			"GMAC0_ETH_CFG3"		=> 0x54,
			"GMAC0_ETH_CFG4"		=> 0x58,
			"GMAC0_ETH_CFG5"		=> 0x5c,
		},
	},
	"gmac1" => {
		readcb				=> \&DumpRegs,
		base				=> 0x1A000000,
		regsdef => {
			"GMAC1_MAC_CFG_1"		=> 0x00,
			"GMAC1_MAC_CFG_2"		=> 0x04,
			"GMAC1_IPG_IFG"			=> 0x08,
			"GMAC1_HALF_DUPLEX"		=> 0x0c,
			"GMAC1_MAX_FRM_LEN"		=> 0x10,
			"GMAC1_MII_CONFIG"		=> 0x20,
			"GMAC1_MII_COMMAND"		=> 0x24,
			"GMAC1_MII_ADDRESS"		=> 0x28,
			"GMAC1_MII_CONTROL"		=> 0x2c,
			"GMAC1_MII_STATUS"		=> 0x30,
			"GMAC1_MII_INDICATOR"		=> 0x34,
			"GMAC1_IFACE_CONTROL"		=> 0x38,
			"GMAC1_IFACE_STATUS"		=> 0x3c,
			"GMAC1_STA_ADDRESS1"		=> 0x40,
			"GMAC1_STA_ADDRESS2"		=> 0x44,
			"GMAC1_ETH_CFG0"		=> 0x48,
			"GMAC1_ETH_CFG1"		=> 0x4c,
			"GMAC1_ETH_CFG2"		=> 0x50,
			"GMAC1_ETH_CFG3"		=> 0x54,
			"GMAC1_ETH_CFG4"		=> 0x58,
			"GMAC1_ETH_CFG5"		=> 0x5c,
		},
	},
	# STEREO registers definition
	"stereo" => {
		readcb				=> \&DumpRegs,
		base				=> 0x180B0000,
		regsdef => {
			"STEREO_CONFIG"			=> 0x0,
			"STEREO_VOLUME"			=> 0x4,
			"STEREO_MASTER_CLOCK"		=> 0x8,
			"STEREO_TX_SAMPLE_CNT_LSB"	=> 0xC,
			"STEREO_TX_SAMPLE_CNT_MSB"	=> 0x10,
			"STEREO_RX_SAMPLE_CNT_LSB"	=> 0x14,
			"STEREO_RX_SAMPLE_CNT_MSB"	=> 0x18,
		},
	},
	# MBOX registers definition
	"mbox" => {
		readcb				=> \&DumpRegs,
		base				=> 0x180A0000,
		regsdef => {
			"MBOX_FIFO_STATUS"		=> 0x8,
			"SLIC_MBOX_FIFO_STATUS"		=> 0xC,
			"MBOX_DMA_POLICY"		=> 0x10,
			"SLIC_MBOX_DMA_POLICY"		=> 0x14,
			"MBOX0_DMA_RX_DESCRIPTOR_BASE"	=> 0x18,
			"MBOX0_DMA_RX_CONTROL"		=> 0x1C,
			"MBOX0_DMA_TX_DESCRIPTOR_BASE"	=> 0x20,
			"MBOX0_DMA_TX_CONTROL"		=> 0x24,
			"MBOX1_DMA_RX_DESCRIPTOR_BASE"	=> 0x28,
			"MBOX1_DMA_RX_CONTROL"		=> 0x2C,
			"MBOX1_DMA_TX_DESCRIPTOR_BASE"	=> 0x30,
			"MBOX1_DMA_TX_CONTROL"		=> 0x34,
			"MBOX_FRAME"			=> 0x38,
			"SLIC_MBOX_FRAME"		=> 0x3C,
			"FIFO_TIMEOUT"			=> 0x40,
			"MBOX_INT_STATUS"		=> 0x44,
			"SLIC_MBOX_INT_STATUS"		=> 0x48,
			"MBOX_INT_ENABLE"		=> 0x4C,
			"SLIC_MBOX_INT_ENABLE"		=> 0x50,
			"MBOX_FIFO_RESET"		=> 0x54,
			"SLIC_MBOX_FIFO_RESET"		=> 0x58,
		},
	},
	"otp" => {
		readcb				=> \&DumpOTP,
		base				=> 0x18130000,
		read_addr			=> 0x1813101c,
		size				=> 1024,
	},
);

# PrintUsage
# Strips out the dir in the original command and returns a help message
# Note that this function doesn't print anything; it expects the caller to do it
sub PrintUsage() {
	my $s_cmd;
	$0 =~ /^.*?([^\/\\+\s]+)(\s|$)/; $s_cmd = $1;
	return "$s_cmd:\n".
		"    -s|--stereo  dump STEREO registers\n".
		"    -m|--mbox    dump MBOX registers\n".
		"    -g|--gpio    dump GPIO registers\n".
		"    -e|--gmac    dump GMAC (Ethernet) generic registers\n".
		"    -g0|--gmac0  dump GMAC0 registers\n".
		"    -g1|--gmac1  dump GMAC0 registers\n".
		"    -o|--otp     dump OTP registers\n".
		"    -h|--help    print this help\n".
		"";
}

# ParseArgs
# \@arg1: Configuration structure
#         This structure will be used to store the argument processing result
# This function processes the cmdline argument and stores it into the hash
# provided as an argument
sub ParseArgs($) {
	my($h_conf) = @_;
	my($i) = 0;

	if(!exists($$h_conf{dump})) {
		$$h_conf{dump} = [];
	}

	die &PrintUsage() if ($#ARGV < 0);
	while ($i <= $#ARGV) {
		my $arg = $ARGV[$i++];
		if ($arg eq "-s" or $arg eq "--stereo") {
			push(@{$$h_conf{dump}},"stereo");
		}
		elsif ($arg eq "-m" or $arg eq "--mbox") {
			push(@{$$h_conf{dump}},"mbox");
		}
		elsif ($arg eq "-g" or $arg eq "--gpio") {
			push(@{$$h_conf{dump}},"gpio");
		}
		elsif ($arg eq "-e" or $arg eq "--gmac") {
			push(@{$$h_conf{dump}},"gmac");
		}
		elsif ($arg eq "-g0" or $arg eq "--gmac0") {
			push(@{$$h_conf{dump}},"gmac0");
		}
		elsif ($arg eq "-g1" or $arg eq "--gmac1") {
			push(@{$$h_conf{dump}},"gmac1");
		}
		elsif ($arg eq "-o" or $arg eq "--otp") {
			push(@{$$h_conf{dump}},"otp");
		}
		elsif ($arg eq "-h" or $arg eq "--help") {
			print &PrintUsage();
			exit(0);
		}
		else {
			die "Unrecognized arg \"$arg\".\n" . &PrintUsage();
		}
	}
}

sub DumpRegs($) {
	my($h_regs) = shift;
	my $base_addr = $h_regs->{"base"};
	my $h_regsdef = $h_regs->{"regsdef"};
	my @t_regnames;

	@t_regnames = keys %{$h_regsdef};
	@t_regnames = sort {$h_regsdef->{$a} <=> $h_regsdef->{$b} } @t_regnames;

	foreach my $regname (@t_regnames) {
		my $cmd = sprintf("devmem2 0x%08x w", $base_addr + $h_regsdef->{$regname});
		my @retval = `$cmd`;
		# Process the output from devmem2
		$retval[2] =~ s/.*Value at address 0x.*: 0x(.*)\n/$1/;
		printf("%-39s 0x%08s\n",$regname,$retval[2]);
	}
}

sub DumpOTP($) {
	my($h_regs) = shift;
	my $base_addr = $h_regs->{"base"};
	my $read_addr = $h_regs->{"read_addr"};

	# We need to init these registers first to make the OTP readable
	$cmd = sprintf("devmem2 0x18131008 w 0xc8") and `$cmd`;
	$cmd = sprintf("devmem2 0x18131024 w 0x1") and `$cmd`;
	$cmd = sprintf("devmem2 0x1813102c w 0x1") and `$cmd`;
	$cmd = sprintf("devmem2 0x18131000 w 0x010ad079") and `$cmd`;

	foreach my $reg (0..$h_regs->{"size"} - 1) {
		my $cmd;
		next if ($reg % 4 != 0);
		# Ok, we perform the OTP read process here
		# 1st, we read the address we want to access - it returns BADC0FEE
		$cmd = sprintf("devmem2 0x%08x w", $base_addr + $reg) and `$cmd`;
		# 2nd, we read the value at the preset address - this should now
		# contained the value fetched in the above step 
		my $cmd = sprintf("devmem2 0x%08x w", $h_regs->{"read_addr"});
		my @retval = `$cmd`;
		# Process the output from devmem2
		$retval[2] =~ s/.*Value at address 0x.*: 0x(.*)\n/$1/;
		printf("%-39x 0x%08s\n",$base_addr + $reg,$retval[2]);
	}
}

sub Main($) {
	my($h_conf) = @_;
	foreach my $h_regs_name (@{$$h_conf{dump}}) {
		my $h_regs = $h_registers{$h_regs_name};
		printf("----------------------------------------\n");
		printf("    %-31s (base:%08x)\n",uc($h_regs_name),
			$h_regs->{base});
		$h_regs->{readcb}($h_regs);
	}
}

&ParseArgs(\%h_Config);
&Main(\%h_Config);
